#include <globals.h>
#include <usbcHandler.h>
Adafruit_HUSB238 husb238;
bool charging = false;
bool pdRequested = false;
unsigned long lastCallTime_pd = 0;
unsigned long lastCallTime_spin = 0;
unsigned long last_Ping_time = 0;
void chargingSetup() {
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW);
}

void validChargerSpin() {
    HUSB238_VoltageSetting srcVoltage = husb238.getPDSrcVoltage();
    HUSB238_CurrentSetting srcCurrent = husb238.getPDSrcCurrent();
    if (DEBUG) {
        MYSERIAL.println(srcVoltage);
        switch (srcVoltage) {
        case PD_5V:
            MYSERIAL.println("5V detected");
            break;
        case PD_9V:
            MYSERIAL.println("9V detected");
            break;
        case PD_12V:
            MYSERIAL.println("12V detected");
            break;
        case PD_15V:
            MYSERIAL.println("15V detected");
            break;
        case PD_18V:
            MYSERIAL.println("18V detected");
            break;
        case PD_20V:
            MYSERIAL.println("20V detected");
            break;
        case PD_NOT_SELECTED:
            MYSERIAL.println("Not selected");
            break;
        default:
            MYSERIAL.println("Unknown voltage detected");
            break;
        }
        switch (srcCurrent) {
        case CURRENT_0_5_A:
            MYSERIAL.println("0.5A detected");
            break;
        case CURRENT_0_7_A:
            MYSERIAL.println("0.7A detected");
            break;
        case CURRENT_1_0_A:
            MYSERIAL.println("1.0A detected");
            break;
        case CURRENT_1_25_A:
            MYSERIAL.println("1.25A detected");
            break;
        case CURRENT_1_5_A:
            MYSERIAL.println("1.5A detected");
            break;
        case CURRENT_1_75_A:
            MYSERIAL.println("1.75A detected");
            break;
        case CURRENT_2_0_A:
            MYSERIAL.println("2.0A detected");
            break;
        case CURRENT_2_25_A:
            MYSERIAL.println("2.25A detected");
            break;
        case CURRENT_2_75_A:
            MYSERIAL.println("2.75A detected");
            break;
        case CURRENT_3_0_A:
            MYSERIAL.println("3.0A detected");
            break;
        case CURRENT_3_25_A:
            MYSERIAL.println("3.25A detected");
            break;
        case CURRENT_3_5_A:
            MYSERIAL.println("3.5A detected");
            break;
        case CURRENT_4_0_A:
            MYSERIAL.println("4.0A detected");
            break;
        case CURRENT_4_5_A:
            MYSERIAL.println("4.5A detected");
            break;
        case CURRENT_5_0_A:
            MYSERIAL.println("5.0A detected");
            break;
        default:
            MYSERIAL.println("Unknown current detected");
            break;
        }
    }
    if (srcVoltage == requiredVoltage_Stupid_Adafruit && srcCurrent >= requiredCurrent && husb238.isAttached()) {
        charging = true;
        return;
    }
    charging = false;
    pdRequested = false;
}

void usbCspin() {

    if (millis() - lastCallTime_pd < PD_REQUEST_DELAY) {
        return;
    }
    lastCallTime_pd = millis();

    // Don't negotiate if already charging
    if (charging) {
        if (DEBUG)
            MYSERIAL.println("Already charging");
        return;
    }

    // Check if the HUSB238 is connected
    if (!husb238.begin(HUSB238_I2CADDR_DEFAULT, &WIRE)) {
        if (DEBUG)
            Serial.println("HUSB238 not found.");
        return;
    }

    if (DEBUG)
        MYSERIAL.println("HUSB238 found");

    if (!husb238.isAttached())
        return;
    if (DEBUG)
        MYSERIAL.println("HUSB238 is attached");
    if (husb238.getPDResponse() != PD_SUCCESS)
        return;
    if (DEBUG)
        MYSERIAL.println("PD response success");
    if (!husb238.isVoltageDetected(requiredVoltage))
        return;
    if (DEBUG)
        MYSERIAL.println("Required voltage detected");
    // Change to that voltage
    husb238.selectPD(requiredVoltage);
    // Perform the actual PD voltage request!
    husb238.requestPD();

    pdRequested = true;
    if (DEBUG)
        MYSERIAL.println("PD requested");
}

bool moduleConnectedSpin() {
    // Check if anything is listening at the I2C address
    WIRE.beginTransmission(HUSB238_I2CADDR_DEFAULT);
    if (WIRE.endTransmission() == 0) {
        if (DEBUG && false)
            Serial.println("USB C module powered");
        return true;
    }
    if (DEBUG && false)
        Serial.println("USB C module not powered");
    return false;
}
void chargingSpin() {
    // Turns off charging and disconnects the PD if the module is not connected
    if (millis() - last_Ping_time > I2C_LOOP_DELAY) {
        last_Ping_time = millis();
        if (!moduleConnectedSpin()) {
            charging = false;
            pdRequested = false;
            digitalWrite(MOSFET_PIN, LOW);
            return;
        }
    } else {
        return;
    }

    if (millis() - lastCallTime_spin < CHARGE_SPIN_DELAY) {
        return;
    }
    lastCallTime_spin = millis();

    if (DEBUG) {
        MYSERIAL.println("USB C spin");
    }

    usbCspin();

    if (pdRequested) {
        if (DEBUG)
            MYSERIAL.println("Valid charger spin");
        validChargerSpin();
    }

    if (charging) {
        // TODO turn on mosfet
        digitalWrite(MOSFET_PIN, HIGH);
        if (DEBUG)
            MYSERIAL.println("Charging, turning on mosfet");
    } else {
        // TODO turn off mosfet
        digitalWrite(MOSFET_PIN, LOW);
        usbCspin();
        if (DEBUG)
            MYSERIAL.println("Not charging, turning off mosfet");
    }
}