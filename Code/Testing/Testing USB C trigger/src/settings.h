#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <Wire.h>

#define DEBUG false

// I2C setup
#define WIRE Wire
#define SDA_PIN D4
#define SCL_PIN D5

#define MYSERIAL Serial

// Delays
#define LOOP_DELAY 500

// USB C setup
#define requiredVoltage PD_SRC_20V
#define requiredVoltage_Stupid_Adafruit PD_20V
#define requiredCurrent CURRENT_4_5_A

#define PD_REQUEST_DELAY 5000

#define MOSFET_PIN D10
#endif // SETTINGS_H