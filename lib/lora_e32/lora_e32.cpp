#include "lora_e32.h"

// Constructor
LORA_E32::LORA_E32(HardwareSerial &serial, uint8_t M0_pin, uint8_t M1_pin,
                   uint8_t AUX_pin) {
  this->serial = &serial;
  M0 = M0_pin;
  M1 = M1_pin;
  AUX = AUX_pin;

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT);
}

// Initialize the LoRa module with baud rate
void LORA_E32::begin(long baudrate) {
  serial->begin(baudrate);
  setMode(MODE_NORMAL);  // Default to normal mode
}

// Set the mode using the enumerated modes
void LORA_E32::setMode(LORAMode mode) {
  waitForAux();  // Wait for the module to be ready
  switch (mode) {
    case MODE_NORMAL:  // Normal mode
      digitalWrite(M0, LOW);
      digitalWrite(M1, LOW);
      break;
    case MODE_WAKE_UP:  // Wake-up mode
      digitalWrite(M0, HIGH);
      digitalWrite(M1, LOW);
      break;
    case MODE_POWER_SAVING:  // Power-saving mode
      digitalWrite(M0, LOW);
      digitalWrite(M1, HIGH);
      break;
    case MODE_SETUP:  // Sleep mode
      digitalWrite(M0, HIGH);
      digitalWrite(M1, HIGH);
      break;
  }
  delay(100);  // Allow mode to settle
}

// Send data to the module
void LORA_E32::sendData(const uint8_t *data, uint8_t len) {
  waitForAux();              // Wait for the module to be ready
  serial->write(data, len);  // Send data
  waitForAux();              // Wait for the transmission to finish
}

uint8_t LORA_E32::receiveData(uint8_t *buffer, uint8_t len) {
  uint8_t bytesReceived = 0;
  unsigned long startTime = millis();  // Record the start time

  while (bytesReceived < len) {
    // Check if data is available
    if (serial->available()) {
      buffer[bytesReceived++] = serial->read();  // Read a byte of data
    }

    // Check for timeout (10 seconds = 10000 milliseconds)
    if (millis() - startTime >= 5000) {
      Serial.println("Timeout: No data received within 5 seconds.");
      break;  // Exit the loop if timeout occurs
    }
  }

  return bytesReceived;  // Return the number of bytes received
}

// Wait for AUX pin to indicate the module is ready
void LORA_E32::waitForAux() {
  while (digitalRead(AUX) == LOW) {
    delay(1);
  }
}

void LORA_E32::readParameters() {
  uint8_t command[] = {0xC1, 0xC1, 0xC1};  // Read parameters command
  sendData(command, 3);
  delay(50);
  uint8_t buffer[6];  // Expect 6 bytes of response
  if (receiveData(buffer, 6) == 6) {
    printparameter(buffer, 6);
  } else {
    Serial.println("Failed to read parameters.");
  }
}

// Set the parameters for channel, air data rate, and power level
void LORA_E32::setParameters(uint8_t addrh, uint8_t addrl, SPED sped,
                             uint8_t channel, OPTION option) {
  uint8_t command[] = {0xC0, addrh, addrl, sped.byte, channel, option.byte};
  sendData(command, 6);
  delay(50);
  uint8_t buffer[6];
  if (receiveData(buffer, 6) == 6) {
    printparameter(buffer, 6);
  } else {
    Serial.println("Failed to read parameters.");
  }
}

void LORA_E32::reset() {
  uint8_t command[] = {0xC4, 0xC4, 0xC4};  // Command to get version
  sendData(command, 3);                    // Send the command to the module
  delay(50);
};

void LORA_E32::getversion() {
  uint8_t command[] = {0xC3, 0xC3, 0xC3};  // Command to get version
  sendData(command, 3);                    // Send the command to the module
  delay(100);                              // Delay to wait for response

  uint8_t buf[4];  // Expect 3 bytes of response for version
  if (receiveData(buf, 4) == 4) {
    // Process the response
    Serial.println("Module Version:");
    Serial.print("Version Bytes: ");
    for (int i = 0; i < 4; i++) {
      Serial.print("0x");
      if (buf[i] < 0x10) {
        Serial.print("0");
      }
      Serial.print(buf[i], HEX);
      if (i < 3) {
        Serial.print(" ");
      }
    }
    Serial.println();
  } else {
    Serial.println("Failed to get version.");
  }
}

void LORA_E32::printparameter(uint8_t *buffer, uint8_t len) {
  SPED sped;
  OPTION option;

  uint8_t addrh = buffer[1];
  uint8_t addrl = buffer[2];
  sped.byte = buffer[3];
  uint8_t channel = buffer[4];
  option.byte = buffer[5];
  // Print out the parameters for debugging
  Serial.println("\nPrint Parameters:");
  // Serial.println(buffer[0], HEX);
  // Serial.println(buffer[1], HEX);
  // Serial.println(buffer[2], HEX);
  // Serial.println(buffer[3], HEX);
  // Serial.println(buffer[4], HEX);
  // Serial.println(buffer[5], HEX);
  Serial.print("\nAddrH\t\t:\t0x");
  if (addrh < 0x10) {
    Serial.print("0");
  }
  Serial.println(addrh, HEX);
  if (addrl < 0x10) {
    Serial.print("0");
  }
  Serial.print("\nAddrL\t\t:\t0x");
  Serial.println(addrl, HEX);
  Serial.print("\nChannel\t\t:\t");
  Serial.println(channel);

  Serial.println("\nSPED Parameters:");
  Serial.print("Parity Bit\t\t:\t");
  switch (sped.fields.parity) {
    case 0:
      Serial.println("8N1 (no parity-default)");
      break;
    case 1:
      Serial.println("8O1 (odd parity)");
      break;
    case 2:
      Serial.println("8E1 (even parity)");
      break;
    default:
      Serial.println("8N1 (no parity)");
      break;
  }

  Serial.print("UART Baud Rate\t\t:\t");
  switch (sped.fields.baudrate) {
    case 0:
      Serial.println("1200");
      break;
    case 1:
      Serial.println("2400");
      break;
    case 2:
      Serial.println("4800");
      break;
    case 3:
      Serial.println("9600(default)");
      break;
    case 4:
      Serial.println("19200");
      break;
    case 5:
      Serial.println("38400");
      break;
    case 6:
      Serial.println("57600");
      break;
    case 7:
      Serial.println("115200");
      break;
    default:
      break;
  }

  Serial.print("Air Data Rate\t\t:\t");
  switch (sped.fields.airdatarate) {
    case 0:
      Serial.println("0.3 kbps");
      break;
    case 1:
      Serial.println("1.2 kbps");
      break;
    case 2:
      Serial.println("2.4 kbps (Default)");
      break;
    case 3:
      Serial.println("4.8 kbps");
      break;
    case 4:
      Serial.println("9.6 kbps");
      break;
    case 5:
      Serial.println("19.2 kbps");
      break;
    case 6:
      Serial.println("19.2 kbps");
      break;
    case 7:
      Serial.println("19.2 kbps");
      break;
    default:
      break;
  }

  Serial.println("\nOption: ");
  Serial.print("Transmission\t\t:\t");
  switch (option.fields.transmission) {
    case 0:
      Serial.println("Transparent (default)");
      break;
    case 1:
      Serial.println("Fixed mode");
      break;
    default:
      break;
  }

  Serial.print("IOdrive\t\t\t:\t");
  switch (option.fields.iodrive) {
    case 0:
      Serial.println("open collector");
      break;
    case 1:
      Serial.println("Push-pull (default)");
      break;
    default:
      break;
  }

  Serial.print("wakeuptime\t\t:\t");
  switch (option.fields.wakeuptime) {
    case 0:
      Serial.println("250ms (default)");
      break;
    case 1:
      Serial.println("500ms");
      break;
    case 2:
      Serial.println("750ms");
      break;
    case 3:
      Serial.println("1000ms");
      break;
    case 4:
      Serial.println("1250ms");
      break;
    case 5:
      Serial.println("1500ms");
      break;
    case 6:
      Serial.println("1750ms");
      break;
    case 7:
      Serial.println("2000ms");
      break;
    default:
      break;
  }

  Serial.print("FEC\t\t\t:\t");
  switch (option.fields.fec) {
    case 0:
      Serial.println("FED OFF");
      break;
    case 1:
      Serial.println("FEC ON (default)");
      break;
    default:
      break;
  }

  Serial.print("Power\t\t\t:\t");
  switch (option.fields.power) {
    case 0:
      Serial.println("20 dBm (default)");
      break;
    case 1:
      Serial.println("17 dBm");
      break;
    case 2:
      Serial.println("14 dBm");
      break;
    case 3:
      Serial.println("10 dBm");
      break;
    default:
      break;
  }
}
