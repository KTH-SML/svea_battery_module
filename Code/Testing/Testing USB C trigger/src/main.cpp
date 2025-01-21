// #include <neoPixelHandler.h>
#include <currentSensors.h>
#include <debugTools.h>
#include <settings.h>
#include <usbcHandler.h>
void chargingSpin();
void scanI2CBus();
void wireSetup();

void setup() {
    delay(1500);
    // setupNeoPixel();
    // setNeoPixelColor("blue");

    serialSetup();

    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW);

    wireSetup();
    scanI2CBus();

    // resetHUSB238();
    // setNeoPixelColor("cyan");

    // Serial.println("Setup complete");
}

void loop() {
    chargingSpin();
    // Serial.println("Loop running");
    delay(LOOP_DELAY);
}

void chargingSpin() {
    usbCspin();
    if (pdRequested) {
        validChargerSpin();
    }
    if (charging) {
        // TODO turn on mosfet
        digitalWrite(MOSFET_PIN, HIGH);
        // Serial.println("Charging, turning on mosfet");
    } else {
        // TODO turn off mosfet
        digitalWrite(MOSFET_PIN, LOW);
        // Serial.println("Not charging, turning off mosfet");
    }
}

void wireSetup() {
    WIRE.setSCL(SCL_PIN);
    WIRE.setSDA(SDA_PIN);
    WIRE.begin();
}
