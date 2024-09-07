
#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <lora_e32.h>
#include <sht45.h>
#include "SparkFun_SCD30_Arduino_Library.h"

#define MASTER

#define MASTER_ADDR 0
#define DEVICE_ID 1
#ifdef MASTER
#define DEVICE_ADDR MASTER_ADDR
#else
#define DEVICE_ADDR 0x10 + DEVICE_ID
#endif

#define RFCHANNEL 20
#define RFADDRL 0x11
#define RFADDRH 0x44
#define READ_INTERVAL 2500
#define AUX 1
#define M1 2
#define M0 3

LORA_E32 lora(Serial1, M0, M1, AUX);

SHT45 sht45(&Wire, 0x44); // SHT45-AD1B => 0x44
SCD30 airSensor;

typedef union
{
  // uint32_t encoded;  // 32-bit integer representation
  uint8_t buf[5];
  struct
  {
    uint16_t temperatureScaled : 9; // 9 bits for temperature (0-450)
    uint16_t humidityScaled : 10;   // 10 bits for humidity (0-1000)
    uint16_t co2 : 13;              // 13 bits for CO2 (400-5000)
    uint8_t id : 8;
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
// uint8_t addrh = 0xdd;
// uint8_t addrl = 0xee;
uint8_t channel = 0x11;
uint64_t state = 1234567890123456789ULL;
uint64_t randvalue;
uint32_t randtime, nextime;
uint64_t accesstime, t0;
void setup()
{
  Serial.begin(9600);
  lora.begin(9600);
  delay(100);
  Serial.println("init...");
  // Wire.begin();

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
  // initializeSHT45();
  // initializeSCD30(1);
  // delay(2000); // Wait for sensor stabilization
  // configureSCD30();
  // airSensor.setAutoSelfCalibration(1);
  // Serial.println();
  // printSCD30Settings();

  // randvalue = xorshift64(&state);
  // randtime = mapToRange(randvalue, 0, 15000);
  accesstime = millis();
  // getSHT45Data();
  // getSCD30Data();
  t0 = millis();
}

float t45, rh45, ts, rhs;
float td, rhd;
uint16_t co2d;
uint16_t co2;
uint8_t id;
sensordata encdata;
rfpacket txdata;
rfpacket rxdata;

void loop()
{
  // if (millis() - accesstime > randtime)
  // random acces RF transmit
  // txdata = makepacket(RFADDRH, RFADDRL + MASTER_ADDR, RFCHANNEL, encdata);
  // lora.sendData(txdata.rfbuf, 8); /// send over RF
  // printrfbuf(txdata);

  if (lora.receiveData(encdata.buf, 5) == 5)
  {
    printencdata(encdata);
  }
  // memcpy(rxdata.rfbuf, txdata.rfbuf, sizeof(rxdata.rfbuf));
  // decodeData(txdata.rfpacket.payload, td, rhd, co2d, id);
  // randvalue = xorshift64(&state);
  // randtime = mapToRange(randvalue, 250, 15000); // random access time
  // nextime = randtime;
  // printrfdata(txdata);
  // accesstime = millis();
  // }
  delay(10);
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

  rfpacket x;
  x.rfpacket.addrh = addr_h;
  x.rfpacket.addrl = addr_l;
  x.rfpacket.channel = ch;
  x.rfpacket.payload = sd;
  return x;
}