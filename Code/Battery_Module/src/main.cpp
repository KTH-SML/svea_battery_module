#include "currentSensors.h"
#include "debugTools.h"
#include "settings.h"
#include "usbcHandler.h"
#include <globals.h>

void wireSetup() {
    WIRE.setSCL(SCL_PIN);
    WIRE.setSDA(SDA_PIN);
    WIRE.begin();
}

void setup() {
    // serialSetup();
    setup_nh();
    wireSetup();
    if (DEBUG)
        MYSERIAL.println("Setup complete");
}

void loop() {
    // chargingSpin();

    sensorSpin();
}
