/*
 * Author: @github.com/annadostoevskaya
 * Filename: main.cpp
 * Created: 01 Jul 2025 09:29:46
 * Last Update: 28 Aug 2025 05:47:45
 *
 * Description: OPT4003Q1 light sensor test —
 * cover/uncover to validate detection of light and darkness.
 */

#include "Arduino.h"
#include "HardwareSerial.h"
#include "OPT4003Q1.h"

#define OPT4003Q1_ADDR OPT4003Q1_I2C_ADDR_VDD

OPT4003Q1 opt4003q1(true);
OPT4003Q1_Light light = {};

void setup() {
    Serial.begin(9600);

    if (!opt4003q1.begin(OPT4003Q1_ADDR)) {
        if (opt4003q1.getError() == OPT4003Q1_ERROR_I2C_BEGIN_FAILED) {
            Serial.println("[err] OPT4003Q1 i2c begin failed");
        }

        if (opt4003q1.getError() == OPT4003Q1_ERROR_INVALID_DEVICE_ID) {
            Serial.println("[err] OPT4003Q1 invalid DID");
        }

        for (;;) {
            delay(5000);
        }
    }

    Serial.println("[o.k.] OPT4003Q1 initialized");
}

void loop() {
    opt4003q1.enable();

    do {
        delay(100);
    } while (!opt4003q1.isReady());

    light.lux = opt4003q1.getALS();
    if (opt4003q1.getError() == OPT4003Q1_ERROR_CRC_FAILED) {
        Serial.println("[err] crc failed!");
    }

    light.ir = opt4003q1.getIR();
    if (opt4003q1.getError() == OPT4003Q1_ERROR_CRC_FAILED) {
        Serial.println("[err] crc failed!");
    }

    Serial.print("[o.k.] Reading: ");
    Serial.print(light.lux);
    Serial.print(" lux; ");
    Serial.print(light.ir);
    Serial.print(" mW/cm^2");
    Serial.println();

    delay(1000);
}
