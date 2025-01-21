#include <debugTools.h>

void serialSetup() {
    MYSERIAL.begin(115200);
    while (!MYSERIAL && DEBUG)
        delay(10);
    if (!DEBUG)
        return;
    delay(1000);
    MYSERIAL.println("\nSerial Connection Established");
    delay(1000);
}

void scanI2CBus() {
    if (!DEBUG)
        return;
    MYSERIAL.println("Scanning I2C bus...");
    for (byte address = 1; address < 127; address++) {
        WIRE.beginTransmission(address);
        byte error = WIRE.endTransmission();
        if (error == 0) {
            MYSERIAL.print("I2C device found at address 0x");
            if (address < 16)
                MYSERIAL.print("0");
            MYSERIAL.println(address, HEX);
        }
    }
    MYSERIAL.println("I2C scan complete.");
}