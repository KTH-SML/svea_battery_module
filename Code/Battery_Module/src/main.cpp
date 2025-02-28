#include "currentSensors.h"
#include "debugTools.h"
#include "settings.h"
#include "usbcHandler.h"
#include <globals.h>

// void wireSetup() {
//     WIRE.setSCL(SCL_PIN);
//     WIRE.setSDA(SDA_PIN);
//     WIRE.begin();
// }
//
// void setup() {
//     // serialSetup();
//     setup_nh();
//     wireSetup();
//     if (DEBUG)
//         MYSERIAL.println("Setup complete");
// }
//
// void loop() {
//     // chargingSpin();
//
//     sensorSpin();
// }
// #include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();
#define bat_address 0x41
#define charge_address 0x40
void setup() {
    Serial.begin(250000);
    // Wait until serial port is opened
    while (!Serial) {
        delay(10);
    }

    Serial.println("Adafruit INA260 Test");

    if (!ina260.begin(bat_address)) {
        Serial.println("Couldn't find INA260 chip");
        while (1)
            ;
    }
    Serial.println("Found INA260 chip");
}

void loop() {
    Serial.print("Current: ");
    Serial.print(ina260.readCurrent());
    Serial.println(" mA");

    Serial.print("Bus Voltage: ");
    Serial.print(ina260.readBusVoltage());
    Serial.println(" mV");

    Serial.print("Power: ");
    Serial.print(ina260.readPower());
    Serial.println(" mW");

    Serial.println();
    delay(1000);
}
