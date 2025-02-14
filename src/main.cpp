#include "HardwareSerial.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <DHT_U.h>

#include <Ethernet.h>
#include <MQTT.h>

#define DHT_PIN 2
#define DHT_TYPE DHT22
#define TSL2561_SENSOR_ID 0x1
#define ACS712_PIN A0

#define DELAY 3000

#define R1 1496.0f
#define R2 200.0f

struct meas {
    float T, H, L, A, V;
};

meas getMeas();

uint32_t start = 0;

constexpr float voltageDivider = (R1 + R2) / R2;

// Voltmeter
Adafruit_ADS1115 ads1115 = {};

// Digital Temperature and Humidity sensor
DHT_Unified dht(DHT_PIN, DHT_TYPE);

// Luminosity sensor
Adafruit_TSL2561_Unified tsl2561 =
    Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, TSL2561_SENSOR_ID);

EthernetClient net = {};
MQTTClient client;

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {0x00, 0xb0, 0x5a, 0x85, 0x6b, 0x00};

bool connect() {
    return client.connect("arduino-frieren-solar", "public", "public");
}

void setup() {
    Serial.begin(9600);

    /*while (!Serial) {*/
    /*    delay(1);*/
    /*}*/

    /*if (!ads1115.begin()) {*/
    /*    Serial.println("Error: Failed to initialize ADS1115");*/
    /*    while (1) {*/
    /*        delay(1);*/
    /*    }*/
    /*}*/
    /**/
    /*if (!tsl2561.begin()) {*/
    /*    Serial.println("Error: Failed to initialize TSL2561");*/
    /*    while (1) {*/
    /*        delay(1);*/
    /*    }*/
    /*}*/

    /*dht.begin();*/

    Serial.println("Initialize Ethernet with DHCP");
    if (Ethernet.begin(mac) == 0) {
        Serial.println("Failed to configure Ethernet using DHCP");
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
            Serial.println("Ethernet shield was not found.");
        } else if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println("Ethernet cable is not connected.");
        }

        // no point in carrying on, so do nothing forevermore:
        while (1) {
            delay(1);
        }
    }

    // print your local IP address:
    Serial.print("My IP address: ");
    Serial.println(Ethernet.localIP());

    Serial.print("MQTT: connecting");
    client.begin("public.cloud.shift.io", net);
    while (!client.connect("arduino-frieren-solar", "public", "public")) {
        Serial.print(".");
        delay(1000);
    }
}

void loop() {
    if (millis() - start >= DELAY) {

        return;
        client.loop();
        if (!client.connected()) {
            Serial.print("MQTT: connecting");
            while (!connect()) {
                Serial.print(".");
                delay(1000);
            }
        }
        /*start = millis();*/
        /*meas M = getMeas();*/
        /*Serial.print("Voltage: ");*/
        /*Serial.print(M.V);*/
        /*Serial.println(" V");*/
        /*Serial.print("Current: ");*/
        /*Serial.print(M.A);*/
        /*Serial.println(" A");*/
        /*Serial.println("");*/
        /*Serial.print("Temperature: ");*/
        /*Serial.print(M.T);*/
        /*Serial.println("°C");*/
        /*Serial.print("Humidity: ");*/
        /*Serial.print(M.H);*/
        /*Serial.println(" %");*/
        /*Serial.print("Light: ");*/
        /*Serial.print(M.L);*/
        /*Serial.println(" lux");*/
        /*Serial.println("");*/
    }

    switch (Ethernet.maintain()) {
    case 1:
        // renewed fail
        Serial.println("Error: renewed fail");
        break;

    case 2:
        // renewed success
        Serial.println("Renewed success");
        // print your local IP address:
        Serial.print("My IP address: ");
        Serial.println(Ethernet.localIP());
        break;

    case 3:
        // rebind fail
        Serial.println("Error: rebind fail");
        break;

    case 4:
        // rebind success
        Serial.println("Rebind success");
        // print your local IP address:
        Serial.print("My IP address: ");
        Serial.println(Ethernet.localIP());
        break;

    default:
        // nothing happened
        break;
    }
}

meas getMeas() {
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

    return {T, H, L, A, V};
}
