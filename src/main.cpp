#include <Arduino.h>
#include <SHT45.h>
#include <Wire.h>
#define __STM32F4__
#include "LoRa_E32.h"
#include "SparkFun_SCD30_Arduino_Library.h"

void setup() {
  Wire.begin();
  // while (!sht45.begin()) {
  //   Serial.println("SHT45 not found !");
  //   delay(1000);
  // }
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) { return x + y; }