#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <WString.h>
#include <Wire.h>

#include <Ethernet.h>
#include <MQTT.h>
#include <SD.h>

#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ArduinoJson.h>
#include <DHT_U.h>
#include <cstdint>

#define CONFIG_FILENAME   ("CONFIG.TXT")
#define DHT_PIN           (0x2)
#define SD_PIN            (4)
#define DHT_TYPE          (DHT22)
#define TSL2561_SENSOR_ID (0x1)
#define ACS712_PIN        (A0)

#define DELAY             3000

#define R1                1496.0f
#define R2                200.0f

struct meas {
    float T, H, L, A, V;
};

uint32_t start = 0;

constexpr float voltageDivider = (R1 + R2) / R2;

// Voltmeter
Adafruit_ADS1115 ads1115 = {};

// Digital Temperature and Humidity sensor
DHT_Unified dht(DHT_PIN, DHT_TYPE);

// Luminosity sensor
Adafruit_TSL2561_Unified tsl2561 =
    Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, TSL2561_SENSOR_ID);

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {0x00, 0xb0, 0x5a, 0x85, 0x6b, 0x00};

JsonDocument config;

// Networking
EthernetClient net = {};
MQTTClient client;

void getConfig();

inline bool mqttconn() {
    const char *username = config["mqtt_username"];
    const char *password = config["mqtt_password"];
    return client.connect("", username, password);
}

meas getMeas();
void dhcpLoop();

void setup() {
    Serial.begin(9600);

    // TODO: Check this later

    /*while (!Serial) {*/
    /*    delay(1);*/
    /*}*/

    if (!SD.begin(SD_PIN)) {
        Serial.println(F("ERR: SD card initialization failed!"));
        while (1) {
            delay(1);
        }
    }

    getConfig();

    Serial.println("INF: Check `mqtt_host`");
    while (!config["mqtt_host"].is<const char *>()) {
        Serial.println("ERR: `mqtt_host` not found");
    }

    Serial.println("INF: Check `mqtt_username`");
    while (!config["mqtt_username"].is<const char *>()) {
        Serial.println("ERR: `mqtt_username` not found");
    }

    Serial.println("INF: Check `mqtt_password`");
    while (!config["mqtt_password"].is<const char *>()) {
        Serial.println("ERR: `mqtt_password` not found");
    }

    Serial.println("INF: Check `mqtt_topic`");
    while (!config["mqtt_topic"].is<const char *>()) {
        Serial.println("ERR: `mqtt_topic` not found");
    }

    Serial.println("INF: Check `eth_timeout`");
    while (!config["eth_timeout"].is<uint32_t>()) {
        Serial.println("ERR: `eth_timeout` not found");
    }

    Serial.println("INF: Check `eth_response_timeout`");
    while (!config["eth_response_timeout"].is<uint32_t>()) {
        Serial.println("ERR: `eth_response_timeout` not found");
    }

    /*if (!ads1115.begin()) {*/
    /*    Serial.println(F("ERR: Failed to initialize ADS1115"));*/
    /*    while (1) {*/
    /*        delay(1);*/
    /*    }*/
    /*}*/
    /**/

    /*if (!tsl2561.begin()) {*/
    /*    Serial.println(F("ERR: Failed to initialize TSL2561"));*/
    /*    while (1) {*/
    /*        delay(1);*/
    /*    }*/
    /*}*/

    /*dht.begin();*/

    Serial.println(F("INF: Initialize Ethernet with DHCP"));
    if (!Ethernet.begin(mac, config["eth_timeout"].as<uint32_t>(),
                        config["eth_response_timeout"].as<uint32_t>())) {
        Serial.println(F("ERR: Failed to configure Ethernet using DHCP"));
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
            Serial.println(F("ERR: Ethernet shield was not found."));
        } else if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println(F("ERR: Ethernet cable is not connected."));
        }

        // no point in carrying on, so do nothing forevermore:
        while (1) {
            delay(1);
        }
    }

    // print your local IP address:
    const char *mqtthost = config["mqtt_host"];
    Serial.print(F("INF: My IP address "));
    Serial.println(Ethernet.localIP());
    Serial.print(F("INF: MQTT connecting "));
    Serial.println(mqtthost);
    client.begin(mqtthost, net);
    while (!mqttconn()) {
        Serial.print(F("."));
        delay(1000);
    }
}

void loop() {
    char buffer[1024]; // TODO: allocate with malloc()

    client.loop();

    if (!client.connected()) {
        Serial.println("INF: MQTT reconnecting");
        while (!mqttconn()) {
            Serial.print(F("."));
            delay(1000);
        }
    }

    if (millis() - start >= DELAY) {

        start = millis();

        // meas M = getMeas();
        meas M{(float)random(), (float)random(), (float)random(),
               (float)random(), (float)random()};

        JsonDocument payload;
        payload["tem"] = M.T;
        payload["hum"] = M.H;
        payload["lux"] = M.L;
        payload["vol"] = M.V;
        payload["cur"] = M.A;

        size_t sz = serializeJson(payload, buffer);

        if (sz) {
            const char *topic = config["topic"];
            client.publish(topic, buffer);
        }

        Serial.print(F("Voltage: "));
        Serial.print(M.V);
        Serial.println(F(" V"));
        Serial.print(F("Current: "));
        Serial.print(M.A);
        Serial.println(F(" A"));
        Serial.print(F("Temperature: "));
        Serial.print(M.T);
        Serial.println(F("°C"));
        Serial.print(F("Humidity: "));
        Serial.print(M.H);
        Serial.println(F(" %"));
        Serial.print(F("Light: "));
        Serial.print(M.L);
        Serial.println(F(" lux"));
        Serial.println();

        dhcpLoop();
    }
}

void getConfig() {
    Serial.println(F("INF: Reading config file..."));
    if (!SD.exists(CONFIG_FILENAME)) {
        Serial.println(F("ERR: Can't find config file"));
        while (1) {
            delay(1);
        }
    }

    File cfg = SD.open(CONFIG_FILENAME);
    size_t sz = (size_t)cfg.size();
    char *content = (char *)malloc(sz + 1);
    if (!content) {
        Serial.println(F("ERR: Failed to allocate memory"));
        while (1) {
            delay(1);
        }
    }

    content[sz] = '\0';

    if (cfg.read(content, sz) == -1) {
        Serial.println(F("ERR: Failed to read config file."));
        while (1) {
            delay(1);
        }
    }

    Serial.println(content);
    DeserializationError err = deserializeJson(config, content);
    if (err) {
        Serial.print(F("ERR: Deserialization failed, "));
        Serial.println(err.f_str());
        while (1) {
            delay(1);
        }
    }

    free(content);
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

void dhcpLoop() {
    switch (Ethernet.maintain()) {
    case 1:
        // renewed fail
        Serial.println(F("ERR: renewed fail"));
        break;

    case 2:
        // renewed success
        Serial.println(F("INF: Renewed success"));
        // print your local IP address:
        Serial.print(F("INF: My IP address "));
        Serial.println(Ethernet.localIP());
        break;

    case 3:
        // rebind fail
        Serial.println(F("ERR: rebind fail"));
        break;

    case 4:
        // rebind success
        Serial.println(F("INF: Rebind success"));
        // print your local IP address:
        Serial.print(F("INF: My IP address "));
        Serial.println(Ethernet.localIP());
        break;

    default:
        // nothing happened
        break;
    }
}
