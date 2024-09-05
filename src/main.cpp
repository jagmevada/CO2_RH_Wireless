
#include <Arduino.h>
#include <Wire.h>
#include <lora_e32.h>
#include <sht45.h>

#include "SparkFun_SCD30_Arduino_Library.h"
#define AUX 1
#define M1 2
#define M0 3

LORA_E32 lora(Serial1, M0, M1, AUX);

SHT45 sht45(&Wire, 0x44);  // SHT45-AD1B => 0x44
SCD30 airSensor;
// Function declarations
void initializeSHT45();
void initializeSCD30(bool asc);
void configureSCD30();
void printSCD30Settings();
void getSHT45Data();
void getSCD30Data();
void printSensorData();
void setuplora();
// #define
// HardwareSerial pc = Serial;
// HardwareSerial lora = Serial1;
// HardwareSerial Serial;
// HardwareSerial Serial1;
// pc = &Serial;
uint8_t addrh = 0xdd;
uint8_t addrl = 0xee;
uint8_t channel = 0x11;

void setup() {
  Serial.begin(9600);
  lora.begin(9600);
  delay(100);
  Serial.println("init...");
  Wire.begin();

  // Set the module to normal mode
  lora.reset();
  lora.setMode(MODE_SETUP);
  lora.readParameters();
  // setuplora();

  // lora.setMode(MODE_NORMAL);
  delay(10);
  lora.getversion();

  initializeSHT45();
  initializeSCD30(1);
  delay(2000);  // Wait for sensor stabilization
  configureSCD30();
  // airSensor.setAutoSelfCalibration(1);
  printSCD30Settings();
}
float t45, rh45, ts, rhs;
uint16_t co2;
void loop() {
  getSHT45Data();
  getSCD30Data();
  printSensorData();
  delay(2000);
}

// put function definitions here:
// int myFunction(int x, int y) { return x + y; }

// Function definitions
void initializeSHT45() {
  if (!sht45.begin()) {
    Serial.println("SHT45 not found!");
    while (1);  // Halt execution
  }
}

void initializeSCD30(bool asc) {
  if (asc) {
    if (airSensor.begin(Wire, true) == false) {
      Serial.println(
          "Air sensor not detected. Please check wiring. Freezing...");
      while (1);
    }
  } else {
    if (!airSensor.begin()) {
      Serial.println(
          "Air sensor not detected. Please check wiring. Freezing...");
      while (1);  // Halt execution
    }
  }
}

void configureSCD30() {
  // airSensor.setAutoSelfCalibration(1);
  airSensor.setAmbientPressure(995);
  delay(10);
  airSensor.setAltitudeCompensation(75);
  delay(10);
}

void printSCD30Settings() {
  uint16_t settingVal;

  if (airSensor.getForcedRecalibration(&settingVal)) {
    Serial.print("Forced recalibration factor (ppm) is ");
    Serial.println(settingVal);
  } else {
    Serial.println("getForcedRecalibration failed! Freezing...");
    while (1);  // Halt execution
  }

  if (airSensor.getMeasurementInterval(&settingVal)) {
    Serial.print("Measurement interval (s) is ");
    Serial.println(settingVal);
  } else {
    Serial.println("getMeasurementInterval failed! Freezing...");
    while (1);  // Halt execution
  }

  if (airSensor.getTemperatureOffset(&settingVal)) {
    Serial.print("Temperature offset (C) is ");
    Serial.println(((float)settingVal) / 100.0, 2);
  } else {
    Serial.println("getTemperatureOffset failed! Freezing...");
    while (1);  // Halt execution
  }

  if (airSensor.getAltitudeCompensation(&settingVal)) {
    Serial.print("Altitude offset (m) is ");
    Serial.println(settingVal);
  } else {
    Serial.println("getAltitudeCompensation failed! Freezing...");
    while (1);  // Halt execution
  }

  if (airSensor.getFirmwareVersion(&settingVal)) {
    Serial.print("Firmware version is 0x");
    Serial.println(settingVal, HEX);
  } else {
    Serial.println("getFirmwareVersion failed! Freezing...");
    while (1);  // Halt execution
  }

  Serial.print("Auto Self-calibration\t:\t ");
  Serial.println(airSensor.getAutoSelfCalibration() ? "True" : "False");
  Serial.println("\n\n\n");
}

void getSHT45Data() {
  if (sht45.measure()) {
    t45 = sht45.temperature();
    rh45 = sht45.humidity();
  } else {
    Serial.println("SHT45 read error");
  }
}

void getSCD30Data() {
  if (airSensor.dataAvailable()) {
    co2 = airSensor.getCO2();
    ts = airSensor.getTemperature();
    rhs = airSensor.getHumidity();
  } else {
    Serial.println("Waiting for new data");
  }
}

void printSensorData() {
  Serial.print("t45: ");
  Serial.print(String(t45, 1));
  Serial.print(", rh45: ");
  Serial.print(String(rh45, 1));
  Serial.print(", ts: ");
  Serial.print(String(ts, 1));
  Serial.print(", rhs: ");
  Serial.print(String(rhs, 1));
  Serial.print(", co2: ");
  Serial.println(co2);
}

void setuplora() {
  SPED sped;
  OPTION option;
  sped.fields.airdatarate = AIR_DATA_RATE_9_6Kbps;
  sped.fields.baudrate = BAUDRATE_9_6Kbps;
  sped.fields.parity = P_8N1;
  option.fields.transmission = T_Transparent_mode;  ///
  option.fields.iodrive = IO_Default;
  option.fields.wakeuptime = W_t250ms;
  option.fields.fec = FEC_ON;
  option.fields.power = POWER_LEVEL_10dBm;
  uint8_t buffer[6] = {0xc0, addrh, addrl, sped.byte, channel, option.byte};

  Serial.println("\n\nSetparameter:");

  // lora.printparameter(buffer, 6);
  lora.setParameters(addrh, addrl, sped, channel, option);
  // delay(10);

  // delay(100);
  // while (Serial1.available()) {
  //   Serial.println(Serial1.read(), HEX);
  // }
}