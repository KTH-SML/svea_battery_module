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
#define PD_REQUEST_DELAY 1000 / 0.5
#define CHARGE_SPIN_DELAY 1000 / 1
#define CUR_SENSOR_SPIN_DELAY 1000 / 10
#define I2C_LOOP_DELAY 5
// USB C setup
#define requiredVoltage PD_SRC_20V
#define requiredVoltage_Stupid_Adafruit PD_20V
#define requiredCurrent CURRENT_4_5_A
#define requiredCurrent_Stupid_Adafruit 4.5

#define MOSFET_PIN D10

#endif // SETTINGS_H
