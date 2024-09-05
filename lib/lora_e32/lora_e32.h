#ifndef LORA_E32_H
#define LORA_E32_H

#include <Arduino.h>
#define PARITY_SHIFT 6    // PARITY field starts at bit 6
#define BAUDRATE_SHIFT 3  // Baud rate field starts at bit 3
#define AIRDATA_SHIFT 0   // Air data rate field starts at bit 0

// Enumerations for Parity bits
enum Parity { P_8N1 = 0, P_8O1 = 1, P_8E1 = 2 };

// Enumerations for air data rate
enum AirDataRate {
  AIR_DATA_RATE_0_3Kbps = 0,
  AIR_DATA_RATE_1_2Kbps = 1,
  AIR_DATA_RATE_2_4Kbps = 2,
  AIR_DATA_RATE_4_8Kbps = 3,
  AIR_DATA_RATE_9_6Kbps = 4,
  AIR_DATA_RATE_19_2Kbps = 5
};

enum Baudrate {
  BAUDRATE_1_2Kbps = 0,
  BAUDRATE_2_4Kbps = 1,
  BAUDRATE_4_8Kbps = 2,
  BAUDRATE_9_6Kbps = 3,
  BAUDRATE_19_2Kbps = 4,
  BAUDRATE_38_4Kbps = 5,
  BAUDRATE_57_6Kbps = 6,
  BAUDRATE_115_2Kbps = 7
};

enum Transmission { T_Transparent_mode = 0, T_Fixed_mode = 1 };
enum IOdrive { IO_Opencollector = 0, IO_Default = 1 };
enum Wakeuptime {
  W_t250ms = 0,
  W_t500ms = 1,
  W_t750ms = 2,
  W_t1000ms = 3,
  W_t1250ms = 4,
  W_t1500ms = 5,
  W_t1750ms = 6,
  W_t2000ms = 7
};
enum FEC { FEC_OFF = 0, FEC_ON = 1 };
// Enumerations for mode
enum LORAMode {
  MODE_NORMAL = 0,   // without preamble, normal rx
  MODE_WAKE_UP = 1,  // with preamble on tx, normal rx
  MODE_POWER_SAVING =
      2,  // no tx, only sleepmode rx, wakeup by mode1 tx using preamble
  MODE_SETUP = 3
};
// Enumerations for power level
enum PowerLevel {
  POWER_LEVEL_20dBm = 0,
  POWER_LEVEL_17dBm = 1,
  POWER_LEVEL_14dBm = 2,
  POWER_LEVEL_10dBm = 3
};

// Define the bitfield union
union SPED {
  struct {
    uint8_t airdatarate : 3;  // 3 bits for Air Data Rate
    uint8_t baudrate : 3;     // 3 bits for Baud Rate
    uint8_t parity : 2;       // 2 bits for Mode
  } fields;
  uint8_t byte;  // Access the combined byte directly
};

// Define the bitfield union
union OPTION {
  struct {
    uint8_t power : 2;
    uint8_t fec : 1;
    uint8_t wakeuptime : 3;
    uint8_t iodrive : 1;
    uint8_t transmission : 1;
  } fields;
  uint8_t byte;  // Access the combined byte directly
};

class LORA_E32 {
 public:
  // Constructor: Takes in the UART interface, M0, M1, and AUX pins
  LORA_E32(HardwareSerial &serial, uint8_t M0_pin, uint8_t M1_pin,
           uint8_t AUX_pin);

  // Initialize the module with baud rate
  void begin(long baudrate);

  // Set the module mode
  void setMode(LORAMode mode);

  // Send data
  void sendData(const uint8_t *data, uint8_t len);
  //   void setupmode();
  //   void normalmode();
  //   void preamblemode();
  //   void rxsleepmode();
  // Receive data
  uint8_t receiveData(uint8_t *buffer, uint8_t len);

  // Read parameters (frequency, power, etc.)
  void readParameters();

  void setParameters(uint8_t addrh, uint8_t addrl, SPED sped, uint8_t channel,
                     OPTION option);
  void reset();
  void getversion();
  void printparameter(uint8_t *buffer, uint8_t len);

 private:
  HardwareSerial *serial;
  uint8_t M0, M1, AUX;

  // Helper function to handle AUX pin (busy indicator)
  void waitForAux();
};

#endif
