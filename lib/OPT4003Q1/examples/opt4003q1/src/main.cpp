/*
 * Author: @github.com/annadostoevskaya
 * Filename: main.cpp
 * Created: 01 Jul 2025 09:29:46
 * Last Update: 08 Jul 2025 15:19:03
 *
 * Description: <EMPTY>
 */

#include "Arduino.h"
#include "HardwareSerial.h"
#include "OPT4003Q1.h"

#define OPT4003Q1_ADDR OPT4003Q1_I2C_ADDR_VDD

OPT4003Q1 opt4003q1 = {};

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

void loop() {
    opt4003q1.enable();

    Serial.print("INF: OPT4003Q1 waiting measurements");
    while (!opt4003q1.ready()) {
        delay(10);
        Serial.print(".");
    }

    Serial.println();

    OPT4003Q1_Light light = {};
    light.lux = opt4003q1.getALS();
    light.ir = opt4003q1.getIR();

    Serial.print("INF: Result ALS: ");
    Serial.println(light.lux);
    Serial.print("INF: Result IR: ");
    Serial.println(light.ir);
}
