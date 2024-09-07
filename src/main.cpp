
#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <lora_e32.h>
#include <sht45.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include <EEPROM.h>

#define SLAVE

#define ASC_EEPROM_ADDR 0x01
#define MASTER_ADDR 0
#define DEVICE_ID 0
#define SENSOR_ADDR_OFFSET 0x10
#define SENSOR_CHANNEL_OFFSET 0x10

#ifdef MASTER
#define DEVICE_ADDR MASTER_ADDR
#else
#define DEVICE_ADDR SENSOR_ADDR_OFFSET + DEVICE_ID
#define CHANNEL_OFFSET SENSOR_CHANNEL_OFFSET + ((DEVICE_ID) << 1)
#endif

#define RFCHANNEL 20
#define RFADDRL 0x11
#define RFADDRH 0x44
#define READ_INTERVAL 2500

#define ASC 1
#define AUX 1
#define M1 2
#define M0 3

#define CRC16_POLY 0x8005
#define CRC16_INIT 0xFFFF

LORA_E32 lora(Serial1, M0, M1, AUX);

SHT45 sht45(&Wire, 0x44); // SHT45-AD1B => 0x44
SCD30 airSensor;

typedef union
{
  uint8_t cmd;
  struct
  {
    uint8_t checksum : 2; // LSB
    uint8_t asc : 1;
    uint8_t id : 5; // MSB
  } fields;
} command;

typedef union
{
  // uint32_t encoded;  // 32-bit integer representation
  uint8_t buf[5];
  struct
  {
    uint16_t temperatureScaled : 9; // 9 bits for temperature (0-450)
    uint16_t humidityScaled : 10;   // 10 bits for humidity (0-1000)
    uint16_t co2 : 13;              // 13 bits for CO2 (400-5000)
    uint8_t id : 8;                 // command asc_state,  device id and parity
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
  uint8_t rfbuf[8];  // Buffer for raw data, should match the size of datapack
} rfpacket;
// Define a union to overlay a 32-bit integer with the three components

// Function declarations
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
uint8_t even_parity(uint8_t byte);
uint8_t read_asc_eerpom();
void encode_data_with_crc(uint8_t data, uint8_t *encoded);
uint16_t verify_data_with_crc(uint8_t data, const uint8_t *encoded);
uint16_t compute_crc16(uint8_t data);

// uint8_t addrh = 0xdd;
// uint8_t addrl = 0xee;
uint8_t channel = 0x11;
uint64_t state = 1234567890123456789ULL;
uint64_t randvalue;
uint32_t randtime, nextime;
uint64_t accesstime, t0;
uint8_t present_asc_state = 0;
uint8_t asc_state = ASC;
uint8_t eeprom_flashed = 0;
void setup()
{
  Serial.begin(9600);

  asc_state = read_asc_eerpom();
  lora.begin(9600);
  delay(100);
  Serial.println("init...");
  Serial.print("Device ID\t\t:\t");
  Serial.print(DEVICE_ID);
  Wire.begin();

  // Set the module to normal mode
  lora.reset();
  lora.setMode(MODE_SETUP);

  // lora.readParameters();
  setuplora();

  // lora.setMode(MODE_NORMAL);
  delay(10);
  lora.getversion();
  delay(10);
  lora.setMode(MODE_NORMAL);
  delay(10);
  initializeSHT45();
  initializeSCD30(asc_state);
  delay(2000); // Wait for sensor stabilization
  configureSCD30();
  // airSensor.setAutoSelfCalibration(1);
  Serial.println();
  printSCD30Settings();

  // randvalue = xorshift64(&state);
  // randtime = mapToRange(randvalue, 0, 15000);
  // accesstime = millis();
  getSHT45Data();
  getSCD30Data();
  t0 = millis();
}

float t45, rh45, ts, rhs;
float td, rhd;
uint16_t co2d;
uint16_t co2;
uint8_t id;
sensordata encdata;
sensordata cmddata;
rfpacket txdata;
rfpacket rxdata;

void loop()
{
  if (millis() - t0 > READ_INTERVAL)
  { /// independent sensor reading
    t0 = millis();
    getSHT45Data();
    getSCD30Data();
    encdata = encodeData(t45, rh45, co2, 0);
  };

  if (lora.receiveData(cmddata.buf, 5) == 5)
  {
    command check, check1, check0;
    check.cmd = cmddata.parts.id;
    check0.cmd = cmddata.buf[0];
    check1.cmd = cmddata.buf[1];

    if (((check.fields.id == DEVICE_ID) + (check0.fields.id == DEVICE_ID) + (check1.fields.id == DEVICE_ID)) >= 2)
    {
      if (((check.fields.asc != asc_state) + (check0.fields.asc != asc_state) + (check1.fields.asc != asc_state)) >= 2)
      {
        asc_state ^= 1;
        EEPROM.write(ASC_EEPROM_ADDR, asc_state);
        delay(10);
        initializeSCD30(asc_state);
        txdata = makepacket(RFADDRH, RFADDRL + MASTER_ADDR, RFCHANNEL, encdata);
        lora.sendData(txdata.rfbuf, 8); /// send over RF
        delay(2000);                    // Wait for sensor stabilization
        configureSCD30();
        Serial.println();
        printSCD30Settings();
      }
      else
      {
        txdata = makepacket(RFADDRH, RFADDRL + MASTER_ADDR, RFCHANNEL, encdata);
        lora.sendData(txdata.rfbuf, 8); /// send over RF
      }
    }
  }
  // if (millis() - accesstime > randtime)
  // { // random acces RF transmit
  //   txdata = makepacket(RFADDRH, RFADDRL + MASTER_ADDR, RFCHANNEL, encdata);
  //   lora.sendData(txdata.rfbuf, 8); /// send over RF
  //   // printrfbuf(txdata);
  //   // lora.receiveData(encdata.buf,5);
  //   // printencdata(encdata);
  //   // memcpy(rxdata.rfbuf, txdata.rfbuf, sizeof(rxdata.rfbuf));
  //   // decodeData(txdata.rfpacket.payload, td, rhd, co2d, id);
  //   randvalue = xorshift64(&state);
  //   randtime = mapToRange(randvalue, 250, 15000); // random access time
  //   nextime = randtime;
  //   printrfdata(txdata);
  //   accesstime = millis();
  // }
  delay(1);
}

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
  // Print data in one line with labels, commas, and spaces
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
  Serial.print("ppm\tID: ");
  Serial.print(r.rfpacket.payload.parts.id);
  Serial.print("\tt:");
  Serial.println(nextime);
}

void printencdata(sensordata r)
{
  Serial.print("T: ");
  Serial.print(r.parts.temperatureScaled / 10.0, 1);
  Serial.print("°C\tRH: ");
  Serial.print(r.parts.humidityScaled / 10.0, 1);
  Serial.print("%\tCO2: ");
  Serial.print(r.parts.co2);
  Serial.print("ppm\tID: ");
  Serial.println(r.parts.id);
  // Serial.print("\tt:");
  // Serial.println(nextime);
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
  lora.setParameters(RFADDRH, RFADDRL + DEVICE_ADDR, sped, RFCHANNEL + CHANNEL_OFFSET, option); // set my address
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
  sensordata data;
  data.parts.temperatureScaled = tempScaled & 0x1FF; // 9 bits
  data.parts.humidityScaled = humScaled & 0x3FF;     // 10 bits
  data.parts.co2 = co2 & 0x1FFF;                     // 13 bits
  data.parts.id = deviceid;
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
  command tx;
  tx.fields.id = addr_l - (RFADDRL + SENSOR_ADDR_OFFSET);
  tx.fields.asc = asc_state;
  tx.fields.checksum = 0;
  // tx.fields.checksum = even_parity(encdata.buf[0]) ^ even_parity(encdata.buf[1]) ^ even_parity(encdata.buf[2]);
  // tx.fields.checksum += ((even_parity(encdata.buf[3]) ^ even_parity(encdata.buf[4])) << 1);
  sd.parts.id = tx.cmd;
  rfpacket x;
  x.rfpacket.addrh = addr_h;
  x.rfpacket.addrl = addr_l;
  x.rfpacket.channel = ch;
  x.rfpacket.payload = sd;
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
  uint8_t asc_addr = ASC_EEPROM_ADDR;
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

// Function to compute CRC16-CCITT for a single byte of data
uint16_t compute_crc16(uint8_t data)
{
  uint16_t crc = CRC16_INIT;
  crc ^= (uint16_t)data << 8;

  for (uint8_t i = 0; i < 8; ++i)
  {
    if (crc & 0x8000)
    {
      crc = (crc << 1) ^ CRC16_POLY;
    }
    else
    {
      crc = crc << 1;
    }
  }
  return crc;
}

// Function to encode data with CRC16
void encode_data_with_crc(uint8_t data, uint8_t *encoded)
{
  uint16_t crc = compute_crc16(data);
  encoded[0] = (crc >> 8) & 0xFF; // High byte of CRC
  encoded[1] = crc & 0xFF;        // Low byte of CRC
}

// Function to verify data with CRC16
uint16_t verify_data_with_crc(uint8_t data, const uint8_t *encoded)
{
  uint16_t crc = compute_crc16(data);
  uint16_t received_crc = (encoded[0] << 8) | encoded[1];

  return crc == received_crc; // Return 1 if valid, 0 if invalid
}
