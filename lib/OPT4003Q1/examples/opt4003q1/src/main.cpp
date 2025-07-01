/*
 * Author: @github.com/annadostoevskaya
 * Filename: main.cpp
 * Created: 01 Jul 2025 09:29:46
 * Last Update: 01 Jul 2025 11:53:46
 *
 * Description: <EMPTY>
 */

#include "OPT4003Q1.h"

#define OPT4003Q1_ADDR OPT4003Q1_I2C_ADDR_VDD

OPT4003Q1 opt4003q1;

void setup() {
    Serial.begin(9600);

    if (!opt4003q1.begin(OPT4003Q1_ADDR)) {
        Serial.println("ERR: OPT4003Q1 initialization failed!");
        for (;;) {
            delay(5000);
        }
    }

    Serial.println("INF: OPT4003Q1 initialized");
}

void loop() {}
