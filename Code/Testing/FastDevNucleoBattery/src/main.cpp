#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_INA219.h"
#include <math.h> // Include math.h for pow function

// Create INA219 instances
DFRobot_INA219_IIC ina219_battery(&Wire, INA219_I2C_ADDRESS4);
DFRobot_INA219_IIC ina219_charger(&Wire, INA219_I2C_ADDRESS3);

// Calibration parameters
float extMeterReading_mA = 3000;
float ina219Reading_mA_battery = 4685; // Value for fat resistor (battery)
float ina219Reading_mA_charger = 2980; // Value for thin resistor (charger)

// Voltage limits for SoC estimation (for a 3S LiPo battery)
const uint16_t minVoltage = 9000;  // 9.0V
const uint16_t maxVoltage = 12600; // 12.6V

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize the battery INA219 sensor
    while (!ina219_battery.begin()) {
        Serial.println("INA219 battery begin failed");
    }
    // Linear calibration for battery INA219
    ina219_battery.linearCalibrate(ina219Reading_mA_battery, extMeterReading_mA);
    ina219_battery.setPGA(ina219_battery.eIna219PGABits_8);
    ina219_battery.setBADC(ina219_battery.eIna219AdcBits_12, ina219_battery.eIna219AdcSample_32);
    ina219_battery.setSADC(ina219_battery.eIna219AdcBits_12, ina219_battery.eIna219AdcSample_128);

    // Initialize the charger INA219 sensor
    while (!ina219_charger.begin()) {
        Serial.println("INA219 charger begin failed");
    }
    // Linear calibration for charger INA219
    ina219_charger.linearCalibrate(ina219Reading_mA_charger, extMeterReading_mA);
    ina219_charger.setPGA(ina219_charger.eIna219PGABits_8);
    ina219_charger.setBADC(ina219_charger.eIna219AdcBits_12, ina219_charger.eIna219AdcSample_32);
    ina219_charger.setSADC(ina219_charger.eIna219AdcBits_12, ina219_charger.eIna219AdcSample_128);
}

static inline uint8_t sigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
    uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
    return result >= 100 ? 100 : result;
}

static inline uint8_t asigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
    uint8_t result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage), 4.5), 3));
    return result >= 100 ? 100 : result;
}

void printSensorReadings(DFRobot_INA219_IIC& sensor, const char* sensorName) {
    float busVoltage = sensor.getBusVoltage_V();
    uint16_t voltage = static_cast<uint16_t>(busVoltage * 1000); // Convert to mV if needed

    // Calculate SoC using the sigmoidal function
    uint8_t socSigmoidal = sigmoidal(voltage, minVoltage, maxVoltage);

    // Calculate SoC using the asymmetric sigmoidal function
    uint8_t socAsigmoidal = asigmoidal(voltage, minVoltage, maxVoltage);

    // Print the readings
    Serial.print(sensorName); Serial.print(":");
    Serial.print(busVoltage, 4); Serial.print(":");
    Serial.print(sensor.getShuntVoltage_mV() / 1000.0, 4); Serial.print(":");
    Serial.print(sensor.getCurrent_mA() / 1000.0, 3); Serial.print(":");
    Serial.print(sensor.getPower_mW() / 1000.0, 3); 
    if(sensorName == "Battery") {
        Serial.print(":"); Serial.print(socSigmoidal); Serial.print(":");
        Serial.println(socAsigmoidal);
    } else {
        Serial.println();
    }
}

void loop() {
    //Adds a little bit of redundancy if the sensors are not working properly
    if(ina219_battery.lastOperateStatus != ina219_battery.eIna219_ok || ina219_charger.lastOperateStatus != ina219_charger.eIna219_ok) {
        setup();
    }
    printSensorReadings(ina219_battery, "Battery");
    printSensorReadings(ina219_charger, "Charger");
    delay(100);
}