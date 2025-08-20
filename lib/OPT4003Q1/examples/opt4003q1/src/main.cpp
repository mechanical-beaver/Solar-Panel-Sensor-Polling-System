/*
 * Author: @github.com/annadostoevskaya
 * Filename: main.cpp
 * Created: 01 Jul 2025 09:29:46
 * Last Update: 21 Aug 2025 00:53:12
 *
 * Description: OPT4003Q1 light sensor test —
 * cover/uncover to validate detection of light and darkness.
 */

#include "Arduino.h"
#include "HardwareSerial.h"
#include "OPT4003Q1.h"

#define OPT4003Q1_ADDR OPT4003Q1_I2C_ADDR_VDD

OPT4003Q1 opt4003q1(true);

void setup() {
    Serial.begin(9600);

    if (!opt4003q1.begin(OPT4003Q1_ADDR)) {
        Serial.println("[err] OPT4003Q1 initialization failed");
        for (;;) {
            delay(5000);
        }
    }

    Serial.println("[o.k.] OPT4003Q1 initialized");
}

void loop() {
    OPT4003Q1_Light light = {};

    Serial.println("=== TEST: PLEASE COVER THE SENSOR ===");
    delay(2000);

    while (true) {
        opt4003q1.enable();
        do {
            delay(20);
        } while (!opt4003q1.isReady());

        light.lux = opt4003q1.getALS();
        /*light.ir = opt4003q1.getIR();*/

        Serial.print("Reading (covered): ");
        Serial.print(light.lux);
        Serial.print(" lux; ");
        /*Serial.print(light.ir);*/
        /*Serial.print(" mW/cm^2");*/
        Serial.println();

        if (light.lux < 5.0) {
            Serial.println("[o.k.] Low light detected — sensor is covered");
            /*break;*/
        }

        delay(500);
    }

    Serial.println("=== TEST: NOW UNCOVER THE SENSOR ===");
    delay(2000);

    while (true) {
        opt4003q1.enable();

        do {
            delay(20);
        } while (!opt4003q1.isReady());

        light.lux = opt4003q1.getALS();
        /*light.ir = opt4003q1.getIR();*/

        Serial.print("Reading (uncovered): ");
        Serial.print(light.lux);
        Serial.print(" lux; ");
        /*Serial.print(light.ir);*/
        /*Serial.print(" mW/cm^2");*/
        Serial.println();

        if (light.lux > 50.0) {
            Serial.println("[o.k.] Light detected — sensor is uncovered");
            break;
        }

        delay(500);
    }

    Serial.println("[o.k.] TEST PASSED SUCCESSFULLY");

    // Optional pause before repeating
    delay(100000);
}
