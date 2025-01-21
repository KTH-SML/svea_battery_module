#include "Adafruit_HUSB238.h"
#include <Arduino.h>
#include <Wire.h>
#include <settings.h>

Adafruit_HUSB238 husb238;

// Global Variables
bool charging = false;
bool pdRequested = false;
unsigned long lastCallTime = 0;

void validChargerSpin() {
    HUSB238_VoltageSetting srcVoltage = husb238.getPDSrcVoltage();
    HUSB238_CurrentSetting srcCurrent = husb238.getPDSrcCurrent();
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
    if (srcVoltage == requiredVoltage_Stupid_Adafruit && srcCurrent == requiredCurrent && husb238.isAttached()) {
        charging = true;
        return;
    }
    charging = false;
    pdRequested = false;
}

void usbCspin() {

    if (millis() - lastCallTime < PD_REQUEST_DELAY) {
        return;
    }
    lastCallTime = millis();

    // Don't negotiate if already charging
    if (charging) {
        MYSERIAL.println("Already charging");
        return;
    }

    // Check if the HUSB238 is connected
    if (!husb238.begin(HUSB238_I2CADDR_DEFAULT, &WIRE)) {
        Serial.println("HUSB238 not found.");
        return;
    }
    MYSERIAL.println("HUSB238 found");
    if (!husb238.isAttached())
        return;
    MYSERIAL.println("HUSB238 is attached");
    if (husb238.getPDResponse() != PD_SUCCESS)
        return;
    MYSERIAL.println("PD response success");
    if (!husb238.isVoltageDetected(requiredVoltage))
        return;
    MYSERIAL.println("Required voltage detected");
    // Change to that voltage
    husb238.selectPD(requiredVoltage);
    delay(100);
    // Perform the actual PD voltage request!
    husb238.requestPD();
    delay(100);
    pdRequested = true;
    MYSERIAL.println("PD requested");
}