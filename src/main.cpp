
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <SD.h>
#include <Wire.h>
#include <lora_e32.h>
#include <sht45.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include <EEPROM.h>

// ###################################################################
// ###################### DEFINED CONSTANTS ##########################
// ###################################################################

#define MASTER
#define DEPLOYED_SENSOR 25
#define MASTER_ADDR 0
#define DEVICE_ID 0
#define SENSOR_CHANNEL_OFFSET 0x10
#define SENSOR_ADDR_OFFSET 0X10

#ifdef MASTER
#define DEVICE_ADDR MASTER_ADDR
#else
#define DEVICE_ADDR 0x10 + DEVICE_ID
#endif

#define SETUPTIMEOUT 5000

#define RFCHANNEL 0x05
#define RFADDRL 0x11
#define RFADDRH 0x44
#define READ_INTERVAL 2500

#define FRC 0
#define ASC 1
#define AUX 1
#define M1 2
#define M0 3

// CRC-8 polynomial (0x07 or 0x1D for other common CRC-8 variants)
#define CRC8_POLY 0x07

// ###################################################################
// ###################### HARDWARE INITIALIZATION ####################
// ###################################################################

LORA_E32 lora(Serial1, M0, M1, AUX);

SHT45 sht45(&Wire, 0x44); // SHT45-AD1B => 0x44
SCD30 airSensor;

// ###################################################################
// ###################### DATA TYPES #################################
// ###################################################################
typedef uint8_t u8;
typedef uint16_t u16;

typedef union
{
  uint8_t buf[8];
  struct
  {
    u8 status : 8;
    u8 address : 8;
    u16 t : 16;
    u16 rh : 16;
    u16 co2 : 16;
  } data;
} sensor;

sensor frame[DEPLOYED_SENSOR]; // Number of DEPLOYED_SENSORS
u8 framebuf[DEPLOYED_SENSOR * 8];
typedef union
{
  uint8_t cmd;
  struct
  {
    uint8_t frc : 1; // LSB
    uint8_t asc : 1;
    uint8_t id : 6; // MSB
  } fields;
} command;

typedef union
{
  // uint32_t encoded;  // 32-bit integer representation
  uint8_t buf[6];
  struct
  {
    uint16_t temperatureScaled : 9; // 9 bits for temperature (0-450)
    uint16_t humidityScaled : 10;   // 10 bits for humidity (0-1000)
    uint16_t co2 : 13;              // 13 bits for CO2 (400-5000)
    uint8_t id : 8;                 // command asc_state,  device id and parity
    uint8_t crc : 8;
  } parts;
} sensordata;

typedef struct
{
  uint8_t addrh;
  uint8_t addrl;
  uint8_t channel;
  sensordata payload; // The sensor data is part of this structure
} datapack;

typedef union rfdata
{
  datapack rfpacket; // Structured packet
  uint8_t rfbuf[9];  // Buffer for raw data, should match the size of datapack
} rfpacket;
// Define a union to overlay a 32-bit integer with the three components

// ###################################################################
// ###################### FUNCTIONS ###################################
// ####################################################################

void initializeSHT45();
void initializeSCD30(bool asc);
void configureSCD30();
void printSCD30Settings();
void getSHT45Data();
void getSCD30Data();
void printSensorData();
void setuplora();
void decodeData(sensordata encd, float &temp, float &rh, uint16_t &co2, uint8_t &id);
sensordata encodeData(float temperature, float humidity, uint16_t co2, uint8_t id);
rfpacket makepacket(uint8_t addr_h, uint8_t addr_l, uint8_t ch, sensordata sd);
void printrfdata(rfpacket r);
uint64_t xorshift64(uint64_t *state);
uint32_t mapToRange(uint64_t randomValue, uint32_t min, uint32_t max);
void printrfbuf(rfdata xdata);
void printencdata(sensordata r);
uint8_t even_parity(uint8_t byte); // even parity check
uint8_t read_asc_eerpom();
uint8_t compute_crc8(const uint8_t *data, size_t length);
sensordata setFRCSettings();
String readStringFromSerial();
void setup_timer_b0();
void setmode();
void setup_timer_b0();
void disable_watchdog();
void enable_watchdog(uint8_t clk); // 0-off, 1-8clk, 2-16clk,3-32clk,4-64clk ...A-1024clk, clk=1khz
void reset();
void checkinput();
uint8_t confirmyesno();

// ###################################################################
// ############## GLOBAL VARIABLES ###################################
// ###################################################################

uint8_t channel = 0x11;
uint64_t state = 1234567890123456789ULL;
uint64_t randvalue;
uint32_t randtime, nextime;
uint64_t accesstime, t0, setuptime;
uint8_t asc_state = ASC;
uint8_t frc_state = FRC;
uint8_t eeprom_flashed = 0;
uint8_t setting_enabled = 0;
uint8_t smode = 1;
float t45, rh45, ts, rhs;
float td, rhd;
uint16_t co2d;
uint16_t co2;
uint8_t id;
sensordata encdata;
rfpacket txdata;
rfpacket rxdata;
uint8_t parity[5];

const sensordata zero = {
    .parts = {
        .temperatureScaled = 0,
        .humidityScaled = 0,
        .co2 = 0,
        .id = 0,
        .crc = 0}};

const sensor sensorzero = {
    .data = {
        .status = 1,
        .address = 0,
        .t = 0,
        .rh = 0,
        .co2 = 0}};
// ###################################################################
// #######################  SETUP  ###################################
// ###################################################################

void setup()
{
  disable_watchdog();
  Serial.begin(9600);
  asc_state = read_asc_eerpom();
  frc_state = FRC;
  lora.begin(9600);
  delay(100);
  Serial.println("init...");
  // Wire.begin();
  // Set the module to normal mode
  lora.reset();
  lora.setMode(MODE_SETUP);
  asc_state = ASC;
  frc_state = FRC;
  // lora.readParameters();
  setuplora();

  // lora.setMode(MODE_NORMAL);
  delay(10);
  lora.getversion();
  delay(10);
  lora.setMode(MODE_NORMAL);
  delay(200);
  t0 = millis();
  checkinput();
}

// ###################################################################
// #######################  LOOP#  ###################################
// ###################################################################

void loop()
{

  if (setting_enabled)
  {
    smode = 1;
    while (smode)
    {
      Serial.println("Entering setting mode");
      setmode();
    }
  }
  // uint8_t i = 6;
  for (uint8_t i = 1; i <= DEPLOYED_SENSOR; i++) // DEPLOYED_SENSOR
  {
    encdata = encodeData(t45, rh45, co2, 0);
    Serial.flush();
    txdata = makepacket(RFADDRH, RFADDRL + SENSOR_ADDR_OFFSET + i, RFCHANNEL + i, encdata);
    lora.sendData(txdata.rfbuf, 9); /// send over RF
                                    // Serial.println();
    sensor sx;
    sx = sensorzero;
    frame[i - 1] = sx;
    delay(300);
    // Serial.print("TX: ");
    // Serial.print(i);
    if (lora.receiveData(encdata.buf, 6) == 6)
    {
      uint8_t crc = compute_crc8(encdata.buf, 6);
      if (crc == 0)
      {
        command cx;
        cx.cmd = encdata.parts.id;
        sx.data.address = cx.fields.id;
        sx.data.status = 0;
        sx.data.rh = encdata.parts.humidityScaled;
        sx.data.co2 = encdata.parts.co2;
        sx.data.t = encdata.parts.temperatureScaled;
        u8 index = cx.fields.id - 1;
        frame[index] = sx;
        // Serial.print("\tRX: ");
        // Serial.print(cx.fields.id);
        // Serial.print("\tCRC: ");
        // Serial.print((int)crc);
        // Serial.print("\t");
        // printencdata(encdata);
      }
    }
    else
      ;
    // Serial.println();
  }
  memcpy(framebuf, &frame[0], sizeof(framebuf));
  Serial.write(framebuf, DEPLOYED_SENSOR * sizeof(sensor));
  // Serial.write(frame, DEPLOYED_SENSOR * sizeof(sensor));
  //  memcpy(rxdata.rfbuf, txdata.rfbuf, sizeof(rxdata.rfbuf));
  //  decodeData(txdata.rfpacket.payload, td, rhd, co2d, id);
  //  randvalue = xorshift64(&state);
  //  randtime = mapToRange(randvalue, 250, 15000); // random access time
  //  nextime = randtime;
  //  printrfdata(txdata);
  //  accesstime = millis();
  //  }
  delay(10);
}

// ###################################################################
// #######################  FUNCTIONS DEFINATION #####################
// ###################################################################

// put function definitions here:
// int myFunction(int x, int y) { return x + y; }

// Function definitions
void initializeSHT45()
{
  if (!sht45.begin())
  {
    Serial.println("SHT45 not found!");
    while (1)
      ; // Halt execution
  }
}

void initializeSCD30(bool asc)
{
  if (asc)
  {
    if (airSensor.begin(Wire, true) == false)
    {
      Serial.println(
          "Air sensor not detected. Please check wiring. Freezing...");
      while (1)
        ;
    }
  }
  else
  {
    if (!airSensor.begin())
    {
      Serial.println(
          "Air sensor not detected. Please check wiring. Freezing...");
      while (1)
        ; // Halt execution
    }
  }
}

void configureSCD30()
{
  // airSensor.setAutoSelfCalibration(1);
  airSensor.setAmbientPressure(995);
  delay(10);
  airSensor.setAltitudeCompensation(75);
  delay(10);
}

void printSCD30Settings()
{
  uint16_t settingVal;

  if (airSensor.getForcedRecalibration(&settingVal))
  {
    Serial.print("Forced recalibration factor (ppm) is ");
    Serial.println(settingVal);
  }
  else
  {
    Serial.println("getForcedRecalibration failed! Freezing...");
    while (1)
      ; // Halt execution
  }

  if (airSensor.getMeasurementInterval(&settingVal))
  {
    Serial.print("Measurement interval (s) is ");
    Serial.println(settingVal);
  }
  else
  {
    Serial.println("getMeasurementInterval failed! Freezing...");
    while (1)
      ; // Halt execution
  }

  if (airSensor.getTemperatureOffset(&settingVal))
  {
    Serial.print("Temperature offset (C) is ");
    Serial.println(((float)settingVal) / 100.0, 2);
  }
  else
  {
    Serial.println("getTemperatureOffset failed! Freezing...");
    while (1)
      ; // Halt execution
  }

  if (airSensor.getAltitudeCompensation(&settingVal))
  {
    Serial.print("Altitude offset (m) is ");
    Serial.println(settingVal);
  }
  else
  {
    Serial.println("getAltitudeCompensation failed! Freezing...");
    while (1)
      ; // Halt execution
  }

  if (airSensor.getFirmwareVersion(&settingVal))
  {
    Serial.print("Firmware version is 0x");
    Serial.println(settingVal, HEX);
  }
  else
  {
    Serial.println("getFirmwareVersion failed! Freezing...");
    while (1)
      ; // Halt execution
  }

  Serial.print("Auto Self-calibration\t:\t ");
  Serial.println(airSensor.getAutoSelfCalibration() ? "True" : "False");
  Serial.println("\n\n\n");
}

void getSHT45Data()
{
  if (sht45.measure())
  {
    t45 = sht45.temperature();
    rh45 = sht45.humidity();
  }
  else
  {
    Serial.println("SHT45 read error");
  }
}

void getSCD30Data()
{
  if (airSensor.dataAvailable())
  {
    co2 = airSensor.getCO2();
    ts = airSensor.getTemperature();
    rhs = airSensor.getHumidity();
  }
  else
  {
    Serial.println("Waiting for new data");
  }
}

void printrfbuf(rfdata xdata)
{
  Serial.print(xdata.rfbuf[0], HEX);
  Serial.print("-");
  Serial.print(xdata.rfbuf[1], HEX);
  Serial.print("-");
  Serial.print(xdata.rfbuf[2], HEX);
  Serial.print("-");
  Serial.print(xdata.rfbuf[3], HEX);
  Serial.print("-");
  Serial.print(xdata.rfbuf[4], HEX);
  Serial.print("-");
  Serial.print(xdata.rfbuf[5], HEX);
  Serial.print("-");
  Serial.print(xdata.rfbuf[6], HEX);
  Serial.print("-");
  Serial.println(xdata.rfbuf[7], HEX);
  Serial.print("-");
  Serial.println(xdata.rfbuf[8], HEX);
}
void printSensorData()
{
  // Serial.print("t45: ");
  // Serial.print(String(t45, 1));
  // Serial.print(", rh45: ");
  // Serial.print(String(rh45, 1));
  // Serial.print(", ts: ");
  // Serial.print(String(ts, 1));
  // Serial.print(", rhs: ");
  // Serial.print(String(rhs, 1));
  // Serial.print(", co2: ");
  // Serial.println(co2);
  Serial.print("td: ");
  Serial.print(String(td, 1));
  Serial.print(", rhd: ");
  Serial.print(String(rhd, 1));
  Serial.print(", ts: ");
  Serial.print(String(ts, 1));
  Serial.print(", rhs: ");
  Serial.print(String(rhs, 1));
  Serial.print(", co2d: ");
  Serial.println(co2d);
  Serial.println();
}

void printrfdata(rfpacket r)
{
  Serial.print("ID: 0x");
  Serial.print(r.rfpacket.addrh, HEX);
  Serial.print(r.rfpacket.addrl, HEX);
  Serial.print(", CH: ");
  Serial.print(r.rfpacket.channel, HEX);
  Serial.print("\tT: ");
  Serial.print(r.rfpacket.payload.parts.temperatureScaled / 10.0, 1);
  Serial.print("°C\tRH: ");
  Serial.print(r.rfpacket.payload.parts.humidityScaled / 10.0, 1);
  Serial.print("%\tCO2: ");
  Serial.print(r.rfpacket.payload.parts.co2);
  command c;
  c.cmd = r.rfpacket.payload.parts.id;
  Serial.print("ppm\tID: ");
  Serial.print(c.fields.id);
  Serial.print("\tASC: ");
  Serial.print(c.fields.asc);
  Serial.print("\tFRC: ");
  Serial.print(c.fields.frc);
  Serial.print("\ttime:");
  Serial.println(nextime);
}

void printencdata(sensordata r)
{
  command x;
  x.cmd = r.parts.id;

  Serial.print("T: ");
  Serial.print(r.parts.temperatureScaled / 10.0, 1);
  Serial.print("°C\tRH: ");
  Serial.print(r.parts.humidityScaled / 10.0, 1);
  Serial.print("%\tCO2: ");
  Serial.print(r.parts.co2);
  Serial.print("ppm\tASC: ");
  Serial.print(x.fields.asc);
  Serial.print("\tFRC: ");
  Serial.print(x.fields.frc);
  Serial.print("\tID: ");
  Serial.println(x.fields.id);
}

void setuplora()
{
  SPED sped;
  OPTION option;
  sped.fields.airdatarate = AIR_DATA_RATE_9_6Kbps;
  sped.fields.baudrate = BAUDRATE_9_6Kbps;
  sped.fields.parity = P_8N1;
  option.fields.transmission = T_Fixed_mode; ///
  option.fields.iodrive = IO_Default;
  option.fields.wakeuptime = W_t250ms;
  option.fields.fec = FEC_ON;
  option.fields.power = POWER_LEVEL_10dBm;
  // uint8_t buffer[6] = {0xc0, RFADDRH, RFADDRL+MASTER_ADDR, sped.byte, RFCHANNEL, option.byte};

  Serial.println("\n\nSetparameter:");

  // lora.printparameter(buffer, 6);
  lora.setParameters(RFADDRH, RFADDRL + DEVICE_ADDR, sped, RFCHANNEL, option); // set my address
  // delay(10);

  // delay(100);
  // while (Serial1.available()) {
  //   Serial.println(Serial1.read(), HEX);
  // }
}

// Function to encode sensor data into a 32-bit integer using the union
sensordata encodeData(float temperature, float humidity, uint16_t co2, uint8_t deviceid)
{
  // Ensure values are within the expected ranges
  if (temperature > 45.0)
    temperature = 0.0;
  if (humidity >= 100.0)
    humidity = 0.0;
  if (co2 > 8190)
    co2 = 8190;

  // Scale and convert the values
  uint16_t tempScaled = (uint16_t)(temperature * 10); // 0-450 (0.0 to 45.0)
  uint16_t humScaled = (uint16_t)(humidity * 10);     // 0-1000 (0.0 to 100.0)

  // Create a union instance for encoding
  sensordata data = zero;
  command cmd;
  cmd.cmd = 0;
  cmd.fields.id = deviceid;

  data.parts.temperatureScaled = tempScaled & 0x1FF; // 9 bits
  data.parts.humidityScaled = humScaled & 0x3FF;     // 10 bits
  data.parts.co2 = co2 & 0x1FFF;                     // 13 bits
  data.parts.id = cmd.cmd;
  return data;
}

// Function to decode a 32-bit integer into sensor data using the union
void decodeData(sensordata encodedData, float &temperature, float &humidity,
                uint16_t &co2, uint8_t &id)
{
  // Create a union instance for decoding
  sensordata data;
  data = encodedData;

  // Extract values from the union
  uint16_t tempScaled = data.parts.temperatureScaled;
  uint16_t humScaled = data.parts.humidityScaled;
  uint16_t co2Scaled = data.parts.co2;

  temperature = tempScaled / 10.0; // Convert back to temperature
  humidity = humScaled / 10.0;     // Convert back to humidity
  co2 = co2Scaled;                 // Convert back to CO2, adding offset
  id = data.parts.id;
}

void sendDataOverSerial1(uint32_t data)
{
  // Create a pointer to the 32-bit data
  const uint8_t *bytePtr = reinterpret_cast<const uint8_t *>(&data);

  // Send each byte of the 32-bit data
  Serial1.write(bytePtr[3]); // Send the highest byte (most significant byte)
  Serial1.write(bytePtr[2]); // Send the second byte
  Serial1.write(bytePtr[1]); // Send the third byte
  Serial1.write(bytePtr[0]); // Send the lowest byte (least significant byte)
}

// 64-bit Xorshift PRNG implementation
uint64_t xorshift64(uint64_t *state)
{
  *state ^= (*state << 13);
  *state ^= (*state >> 7);
  *state ^= (*state << 17);
  return *state;
}

// Map random number to a specific range
uint32_t mapToRange(uint64_t randomValue, uint32_t min, uint32_t max)
{
  return min + (randomValue % (max - min + 1));
}

rfpacket makepacket(uint8_t addr_h, uint8_t addr_l, uint8_t ch, sensordata sd)
{
  uint8_t checksum;
  command tx;
  tx.fields.id = addr_l - (RFADDRL + SENSOR_ADDR_OFFSET);
  tx.fields.asc = asc_state;
  tx.fields.frc = frc_state;

  sd.parts.id = tx.cmd;
  sd.parts.crc = 0;
  sd.buf[5] = compute_crc8(sd.buf, 5);
  rfpacket x;
  x.rfpacket.addrh = addr_h;
  x.rfpacket.addrl = addr_l;
  x.rfpacket.channel = ch;
  x.rfpacket.payload = sd;
  // printrfdata(x);
  return x;
}

// Function to compute even parity for a single byte
uint8_t even_parity(uint8_t byte)
{
  // XOR all bits together
  byte ^= byte >> 4;
  byte ^= byte >> 2;
  byte ^= byte >> 1;
  return byte & 0x01; // Return the least significant bit as the parity
}

uint8_t read_asc_eerpom()
{
  uint8_t asc_addr = 0x01;
  if (EEPROM.read(0xff) == 255)
  {
    eeprom_flashed = 1;
    EEPROM.write(0xff, 1);
    delay(10);
    EEPROM.write(asc_addr, asc_state);
    delay(10);
    Serial.println("EEPROM Flashed");
    return asc_state;
  }
  else
  {
    Serial.println("EEPROM retained");
    return EEPROM.read(asc_addr);
  }
}

// Function to compute CRC-8
uint8_t compute_crc8(const uint8_t *data, size_t length)
{
  uint8_t crc = 0xFF; // Initial value (can be 0x00 or 0xFF depending on the CRC specification)

  for (size_t i = 0; i < length; ++i)
  {
    crc ^= data[i]; // XOR byte into the CRC

    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      if (crc & 0x80)
      {                               // If the MSB is set
        crc = (crc << 1) ^ CRC8_POLY; // Shift and XOR with polynomial
      }
      else
      {
        crc <<= 1; // Just shift
      }
    }
  }
  return crc;
}

// Helper function to read a complete line from Serial
String readStringFromSerial()
{
  String input = "";

  while (true)
  {
    if (Serial.available())
    {
      char c = Serial.read();
      if (c == '\n' || c == '\r')
      {
        // End of line, return the collected input
        return input;
      }
      else
      {
        // Append character to the input string
        input += c;
      }
    }
    // Small delay to avoid busy-waiting
    delay(10);
  }
}

sensordata setFRCSettings()
{
  sensordata sdset = zero;
  command cmd;
  uint8_t validate = 1;
  int sensorID;
  while (validate)
  {
    Serial.print("Enter Sensor ID (1 to ");
    Serial.print(DEPLOYED_SENSOR);
    Serial.print(") or 0 to reset\t: ");
    String idInput = readStringFromSerial();
    Serial.println(idInput);
    sensorID = idInput.toInt();

    if (sensorID > DEPLOYED_SENSOR)
    {
      Serial.print("\nInvalid Sensor ID. Please enter a number between 1 and ");
      Serial.println(DEPLOYED_SENSOR);
    }
    else if (sensorID == 0)
    {
      reset();
    }
    else
      validate = 0; // id validated
  }
  delay(50);
  Serial.flush();

  validate = 1;
  int frcSetting;
  while (validate)
  {
    Serial.print("Enter FRC setting (1 for ON, 0 for OFF)\t: ");
    String frcInput = readStringFromSerial();
    Serial.println(frcInput);

    frcSetting = frcInput.toInt();
    if (frcSetting > 1)
    {
      Serial.println("\nInvalid value");
    }
    else
    {
      cmd.fields.frc = frcSetting;
      validate = 0;
    }
  }
  delay(50);
  Serial.flush();
  int co2Value;
  if (frcSetting == 1)
  {
    validate = 1;
    while (validate)
    {
      Serial.println("Enter 0 to restart or");
      Serial.print("Enter valid co2 value between 390-1500 ppm for forced CO2 recalibration\t: ");
      // Read the CO2 value
      String co2Input = readStringFromSerial();
      Serial.println(co2Input);
      co2Value = co2Input.toInt(); // Convert the string to an integer
      if (co2Value < 390 || co2Value > 1500)
      {
        if (co2Value == 0)
        {
          reset();
        }
        else
        {
          Serial.println("\nInvalid value");
        }
      }
      else
      {
        sdset.parts.co2 = co2Value;
        validate = 0;
      }
    };
  };
  delay(50);
  Serial.flush();
  int ascSetting;
  validate = 1;
  while (validate)
  {
    Serial.print("Enter ASC setting (1 for ON, 0 for OFF)\t: ");
    String ascInput = readStringFromSerial();
    Serial.println(ascInput);
    ascSetting = ascInput.toInt();

    if (ascSetting > 1)
    {
      Serial.println("\nInvalid value");
    }
    else
    {
      cmd.fields.asc = ascSetting;

      validate = 0;
    }
  }
  delay(50);
  Serial.flush();
  cmd.fields.id = sensorID;
  sdset.parts.id = cmd.cmd;
  // Confirm the entered values

  Serial.print("Confirm SensorID\t: ");
  Serial.println(sensorID);

  Serial.print("FRC\t\t\t: ");
  Serial.println(cmd.fields.frc);

  Serial.print("CO2 level\t\t: ");
  Serial.print(sdset.parts.co2);
  Serial.println(" ppm");

  Serial.print("ASC\t\t\t: ");
  Serial.println(cmd.fields.asc);

  Serial.print("Confirm with (Y/N) or Press R to reset\t: ");

  uint8_t confirm = confirmyesno();

  if (!confirm)
  {
    reset();
  }
  Serial.println("Sending setting to sensor...");
  delay(50);
  Serial.flush();
  return sdset;
}

void setmode()
{

  static sensordata sdset = zero;
  command cmd;
  sdset = setFRCSettings();
  // Serial.println("inside setmode, after FRCsetting call");
  sdset.parts.temperatureScaled = 100;
  sdset.parts.humidityScaled = 1000;
  cmd.cmd = sdset.parts.id;
  asc_state = cmd.fields.asc;
  frc_state = cmd.fields.frc;
  Serial.print("TX: ");
  printencdata(sdset);

  if (cmd.fields.id > 0)
  {
    smode = 0;
    Serial.println("Exiting setting mode");
    Serial.println("Press any key to stop monitoring and reset\t: ");
    delay(1000);
    uint8_t count = 1;
    uint8_t validate = 1;
    while (validate)
    {

      txdata = makepacket(RFADDRH, RFADDRL + SENSOR_ADDR_OFFSET + cmd.fields.id, RFCHANNEL + cmd.fields.id, sdset);
      // printrfdata(txdata);
      lora.sendData(txdata.rfbuf, 9); /// send over RF
      // Serial.println();
      Serial.print("sent to : ");
      Serial.println(cmd.fields.id);
      delay(800);
      if (lora.receiveData(encdata.buf, 6) == 6)
      {
        // Serial.print("\tresp ");
        if (count > 0)
          count--;
        Serial.print("RX: ");
        printencdata(encdata);
        delay(50);
        Serial.flush();
        if (count == 0)
        {
          Serial.println("Is reponse ok?");
          Serial.println("Confirm with (Y/N) or Press R to reset:");
          uint8_t confirm = confirmyesno();
          if (confirm == 0) // no
          {
            count = 1;
          }
          else if (confirm == 1) // yes
          {
            count = 5;
            cmd.fields.frc = 0;
            sdset.parts.temperatureScaled = 250;
            sdset.parts.humidityScaled = 500;
            sdset.parts.co2 = 0;
            asc_state = cmd.fields.asc;
            frc_state = 0;
            sdset.parts.id = cmd.cmd;

            Serial.println("disable setmode");
          }
          else // reset
          {
            validate = 0;
          }
        }
      }
    }
    Serial.println("Exited setting mode.");
    smode = 0;
  }
}

void setup_timer_b0() // 100ms
{
  TCB0.CCMP = 50000;                                  // CCMP/1MHz, 10ms
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; //| TCB_RUNSTDBY_bm;
  TCB0.INTFLAGS = TCB_CAPT_bm;                        // Clear any pending interrupt flags
  TCB0.INTCTRL = TCB_CAPT_bm;                         // Enable TCB0 timeout interrupt
};

ISR(TCB0_INT_vect) // 100ms
{                  ///  // active only when awake
  static unsigned int t0 = 50, timercount = t0, xx = 0;
  timercount--;
  if (timercount == 0)
  {
    timercount = t0;
    xx = !xx;
    // digitalWrite(3, xx);
    PORTA_OUT = xx << 7;
    // PORTA_OUTTGL |= PIN7_bm;
  }
  TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
  // sleep_cpu();
}

void disable_watchdog()
{
  cli();
  WDT.CTRLA = 0;
  sei();
}

void enable_watchdog(uint8_t timeout)
{
  cli(); // disable interrupt
  // Set the watchdog timeout period
  // Timeout options: WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS, WDTO_500MS, WDTO_1S, WDTO_2S
  WDT.CTRLA = timeout; // valid 0 to 0xB, 0=off, 8ms, 16ms,... 8s;
  sei();               // enable interrupt
}

void reset()
{
  Serial.println("resetting mcu...");
  enable_watchdog(2);
  while (1)
    ;
  // RSTCTRL.SWRR = 1;
  // sei();
  // enable_watchdog(1);
  // while (1)
  //   ;
}

void checkinput()
{
  smode = 1;
  setting_enabled = 0;
  Serial.println("Press s for setting mode");
  setuptime = millis();
  while (smode)
  {
    if (millis() - setuptime > SETUPTIMEOUT)
    {
      smode = 0;
    }

    if (Serial.available())
    {
      char c = Serial.read();
      if (c == 's' || c == 'S')
      {
        // End of line, return the collected input
        setting_enabled = 1;

        delay(20);

        while (Serial.available())
        {
          Serial.read();
          delay(10);
        }
        smode = 0;
      }
      else if (c == 'q' || c == 'Q')
      {
        smode = 0;
        setting_enabled = 0;
      }
    }
  }
}

uint8_t confirmyesno()
{
  uint8_t validate = 1;
  while (validate)
  {
    String confirmationInput = readStringFromSerial();
    Serial.println(confirmationInput);
    char confirmation = confirmationInput.charAt(0); // Get the first character

    if (confirmation == 'Y' || confirmation == 'y')
    {

      validate = 0;
      return 1;
    }
    else if (confirmation == 'R' || confirmation == 'r')
    {
      // reset();
      validate = 0;
      return 2;
      // Serial.println("FRC setting cancelled.");
    }
    else if (confirmation == 'N' || confirmation == 'n')
    {
      validate = 0;
      return 0;
    }
    else
    {
      Serial.println();
      Serial.println("Invalid value");
    }
    delay(50);
    Serial.flush();
  }
  delay(50);
  Serial.flush();
}
