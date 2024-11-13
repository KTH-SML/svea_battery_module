#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_INA219.h"
#include <math.h> // Include math.h for pow function

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);

// Adjust these parameters based on actual INA219 and multimeter readings for calibration
float extMeterReading_mA = 3000;
float ina219Reading_mA = 2980;

void setup(void) 
{
    Serial.begin(115200);
    while(!Serial);
    
    Serial.println();
    // Initialize the sensor
    while(ina219.begin() != true) {
        Serial.println("INA219 begin failed");
        delay(2000);
    }
    // Linear calibration
    ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);
    ina219.setPGA(ina219.eIna219PGABits_1);
    ina219.setBADC(ina219.eIna219AdcBits_12, ina219.eIna219AdcSample_32);
    ina219.setSADC(ina219.eIna219AdcBits_12, ina219.eIna219AdcSample_128);
    Serial.println();
}

// Sigmoidal function for SoC estimation (taken from https://www.desmos.com/calculator/oyhpsu8jnw)
static inline uint8_t sigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
    uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
    return result >= 100 ? 100 : result;
}

// Asymmetric sigmoidal function for SoC estimation (taken from https://www.desmos.com/calculator/oyhpsu8jnw)
static inline uint8_t asigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
    uint8_t result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage), 4.5), 3));
    return result >= 100 ? 100 : result;
}

void loop(void)
{
    float busVoltage = ina219.getBusVoltage_V();
    uint16_t voltage = static_cast<uint16_t>(busVoltage * 1000); // Convert to mV if needed

    // Set min and max voltage for a 3S LiPo battery
    uint16_t minVoltage = 9000; // 9.0V
    uint16_t maxVoltage = 12600; // 12.6V

    // Calculate SoC using the sigmoidal function
    uint8_t socSigmoidal = sigmoidal(voltage, minVoltage, maxVoltage);

    // Calculate SoC using the asymmetric sigmoidal function
    uint8_t socAsigmoidal = asigmoidal(voltage, minVoltage, maxVoltage);

    // Print the basic readings
    Serial.print("BusVoltage:   ");
    Serial.print(busVoltage, 2);
    Serial.println(" V");

    Serial.print("ShuntVoltage: ");
    Serial.print(ina219.getShuntVoltage_mV(), 3);
    Serial.println(" mV");

    Serial.print("Current:      ");
    Serial.print(ina219.getCurrent_mA(), 1);
    Serial.println(" mA");

    Serial.print("Power:        ");
    Serial.print(ina219.getPower_mW(), 1);
    Serial.println(" mW");

    // Print the SoC estimations
    Serial.print("State of Charge (Sigmoidal): ");
    Serial.print(socSigmoidal);
    Serial.println("%");

    Serial.print("State of Charge (Asymmetric Sigmoidal): ");
    Serial.print(socAsigmoidal);
    Serial.println("%");

    Serial.println("");
    delay(500);
}