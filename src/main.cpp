#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <DHT_U.h>

#include <Ethernet.h>

#define DHT_PIN 2
#define DHT_TYPE DHT22
#define TSL2561_SENSOR_ID 0x1
#define ACS712_PIN A0

#define DELAY 3000

#define R1 1496.0f
#define R2 200.0f

uint32_t start = 0;

constexpr float voltageDivider = (R1 + R2) / R2;

// Voltmeter
Adafruit_ADS1115 ads1115 = {};

// Digital Temperature and Humidity sensor
DHT_Unified dht(DHT_PIN, DHT_TYPE);

// Luminosity sensor
Adafruit_TSL2561_Unified tsl2561 =
    Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, TSL2561_SENSOR_ID);

void setup() {
    Serial.begin(9600);

    if (!ads1115.begin()) {
        Serial.println("Error: Failed to initialize ADS1115");
        while (1) {
        }
    }

    if (!tsl2561.begin()) {
        Serial.println("Error: Failed to initialize TSL2561");
        while (1) {
        }
    }

    dht.begin();
}

void loop() {
    if (millis() - start >= DELAY) {
        start = millis();

        sensors_event_t e;

        dht.temperature().getEvent(&e);
        float T = e.temperature;

        dht.humidity().getEvent(&e);
        float H = e.relative_humidity;

        tsl2561.getEvent(&e);
        float L = e.light;

        float A = (float(analogRead(ACS712_PIN)) * 0.026f) - 0.02f;

        float Vo = (float)ads1115.readADC_SingleEnded(0) * 3.0f / 1000.0f;
        float V = voltageDivider * Vo;

        Serial.print("Voltage: ");
        Serial.print(V);
        Serial.println(" V");
        Serial.print("Current: ");
        Serial.print(A);
        Serial.println(" A");
        Serial.println("");
        Serial.print("Temperature: ");
        Serial.print(T);
        Serial.println("°C");
        Serial.print("Humidity: ");
        Serial.print(H);
        Serial.println(" g/m³");
        Serial.print("Light: ");
        Serial.print(L);
        Serial.println(" lux");
        Serial.println("");
    }
}
