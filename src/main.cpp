#include <stdint.h>
#include <stdlib.h>

#include <Arduino.h>
#include <Dhcp.h>
#include <HardwareSerial.h>
#include <OneWire.h>
#include <SPI.h>
#include <WString.h>
#include <Wire.h>

#include <DallasTemperature.h>
#include <Ethernet.h>
#include <IPAddress.h>
#include <MQTT.h>
#include <OPT4003Q1.h>
#include <SD.h>

#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

#undef OPT4003Q1_I2C_ADDR
#define OPT4003Q1_I2C_ADDR OPT4003Q1_I2C_ADDR_VDD

#define CONFIG_FILENAME    ("CONFIG.TXT")
#define SD_PIN             (4)
#define ACS712_PIN         (A0)

#define ONE_WIRE_PIN       2

#define R1                 150000.0f
#define R2                 14960.0f

struct meas {
    float T, L, A, V;
    double IR;
};

uint32_t start = 0;
uint32_t dt = 0;
// TODO: Rewrite as singleton
char *mqttbuf;

// Voltmeter
constexpr float voltageDivider = (R1 + R2) / R2;
Adafruit_ADS1015 ads1015;

// Temperature
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);

// Luxometer
OPT4003Q1 opt4003q1;

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

    if (!SD.begin(SD_PIN)) {
        Serial.println(F("ERR: SD card initialization failed!"));
        while (1) {
            delay(5000);
        }
    }

    opt4003q1.begin();
    ds18b20.begin();

    if (opt4003q1.begin(OPT4003Q1_I2C_ADDR)) {
        Serial.println("ERR: OPT4003Q1 initialization failed!");
        while (1) {
            delay(1);
        }
    }

    getConfig();

    Serial.println("INF: Check `mqtt_ip`");
    while (!config["mqtt_ip"].is<const char *>()) {
        Serial.println("ERR: `mqtt_ip` not found");
        delay(5000);
    }

    Serial.println("INF: Check `mqtt_port`");
    while (!config["mqtt_port"].is<uint16_t>()) {
        Serial.println("ERR: `mqtt_port` not found");
        delay(5000);
    }

    Serial.println("INF: Check `mqtt_username`");
    while (!config["mqtt_username"].is<const char *>()) {
        Serial.println("ERR: `mqtt_username` not found");
        delay(5000);
    }

    Serial.println("INF: Check `mqtt_password`");
    while (!config["mqtt_password"].is<const char *>()) {
        Serial.println("ERR: `mqtt_password` not found");
        delay(5000);
    }

    Serial.println("INF: Check `mqtt_topic`");
    while (!config["mqtt_topic"].is<const char *>()) {
        Serial.println("ERR: `mqtt_topic` not found");
        delay(5000);
    }

    Serial.println("INF: Check `mqtt_buffer_size`");
    while (!config["mqtt_buffer_size"].is<uint32_t>()) {
        Serial.println("ERR: `mqtt_buffer_size` not found");
        delay(5000);
    }

    Serial.println("INF: Check `dhcp_enable`");
    bool dhcpEnable = config["dhcp_enable"].is<bool>() && config["dhcp_enable"];
    Serial.print("INF: DHCP Client State: ");
    Serial.println(dhcpEnable ? "ENABLE" : "DISABLE");

    if (dhcpEnable) {
        Serial.println("INF: Check `dhcp_timeout`");
        while (!config["dhcp_timeout"].is<uint32_t>()) {
            Serial.println("ERR: `dhcp_timeout` not found");
            delay(5000);
        }

        Serial.println("INF: Check `dhcp_response_timeout`");
        while (!config["dhcp_response_timeout"].is<uint32_t>()) {
            Serial.println("ERR: `dhcp_response_timeout` not found");
            delay(5000);
        }
    } else {
        Serial.println("INF: Check `eth_static_ip`");
        while (!config["eth_static_ip"].is<const char *>()) {
            Serial.println("ERR: `eth_static_ip` not found");
            delay(5000);
        }

        Serial.println("INF: Check `eth_dns`");
        while (!config["eth_dns"].is<const char *>()) {
            Serial.println("ERR: `eth_dns` not found");
            delay(5000);
        }

        Serial.println("INF: Check `eth_gateway`");
        while (!config["eth_gateway"].is<const char *>()) {
            Serial.println("ERR: `eth_gateway` not found");
            delay(5000);
        }

        Serial.println("INF: Check `eth_subnet`");
        while (!config["eth_subnet"].is<const char *>()) {
            Serial.println("ERR: `eth_subnet` not found");
            delay(5000);
        }
    }

    Serial.println("INF: Check `delta_time`");
    while (!config["delta_time"].is<uint32_t>()) {
        Serial.println("ERR: `delta_time` not found");
        delay(5000);
    }

    Serial.println("INF: Check `sp_number`");
    while (!config["sp_number"].is<uint32_t>()) {
        Serial.println("ERR: `sp_number` not found");
        delay(5000);
    }

    dt = config["delta_time"];

    Serial.println("INF: Allocate MQTT buffer");
    mqttbuf = (char *)malloc(config["mqtt_buffer_size"].as<size_t>());
    while (!mqttbuf) {
        Serial.println("ERR: Failed to allocate MQTT buffer");
        delay(5000);
    }

    if (!ads1015.begin()) {
        Serial.println(F("ERR: Failed to initialize ADS1115"));
        while (1) {
            delay(1);
        }
    }

    if (dhcpEnable) {
        Serial.println(F("INF: Initialize Ethernet with DHCP"));
        if (!Ethernet.begin(mac, config["dhcp_timeout"].as<uint32_t>(),
                            config["dhcp_response_timeout"].as<uint32_t>())) {
            Serial.println(F("ERR: Failed to configure Ethernet with DHCP"));
            if (Ethernet.hardwareStatus() == EthernetNoHardware) {
                Serial.println(F("ERR: Ethernet shield was not found."));
            } else if (Ethernet.linkStatus() == LinkOFF) {
                Serial.println(F("ERR: Ethernet cable is not connected."));
            }

            while (1) {
                delay(5000);
            }
        }
    } else {
        Serial.println("INF: Initialize Ethernet with static IP");
        IPAddress ip = {};
        IPAddress dns = {};
        IPAddress gateway = {};
        IPAddress subnet = {};
        ip.fromString(config["eth_static_ip"].as<const char *>());
        dns.fromString(config["eth_dns"].as<const char *>());
        gateway.fromString(config["eth_gateway"].as<const char *>());
        subnet.fromString(config["eth_subnet"].as<const char *>());

        Ethernet.begin(mac, ip, dns, gateway, subnet);
    }

    // print your local IP address:
    IPAddress mqttip = {};
    mqttip.fromString(config["mqtt_ip"].as<const char *>());
    uint16_t mqttport = config["mqtt_port"].as<uint16_t>();
    Serial.print(F("INF: My IP address "));
    Serial.println(Ethernet.localIP());
    Serial.print(F("INF: MQTT connecting "));
    Serial.print(mqttip);
    Serial.print(F(":"));
    Serial.print(mqttport);
    // TODO: Error handling
    client.begin(mqttip, (int)mqttport, net);
    while (!mqttconn()) {
        Serial.print(F("."));
        delay(1000);
    }
}

void loop() {
    client.loop();

    if (!client.connected()) {
        Serial.println("INF: MQTT reconnecting");
        while (!mqttconn()) {
            Serial.print(F("."));
            delay(1000);
        }
    }

    if (millis() - start >= dt) {

        start = millis();

        meas M = getMeas();

        JsonDocument payload;
        payload["sp_number"] = config["sp_number"];
        payload["tem"] = M.T;
        payload["mW"] = M.IR;
        payload["lux"] = M.L;
        payload["vol"] = M.V;
        payload["cur"] = M.A;

        size_t sz =
            serializeJsonPretty(payload, mqttbuf, config["mqtt_buffer_size"]);
        Serial.println(mqttbuf);
        if (sz) {
            const char *topic = config["mqtt_topic"];
            client.publish(topic, mqttbuf, (int)sz);
        }

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
    OPT4003Q1_Light light = {};

    while (true) {
        light.lux = opt4003q1.getALS();
        light.ir = opt4003q1.getIR();

        if (opt4003q1.getError() == OPT4003Q1_ERROR_OK) {
            break;
        }

        Serial.println("err: failed to read light data, retrying...");
        opt4003q1.resetError();
    }

    ds18b20.requestTemperatures();
    float T = ds18b20.getTempCByIndex(0);

    float A = -0.04859 * analogRead(ACS712_PIN) + 24.78;
    float Vo = (float)ads1015.readADC_SingleEnded(0) * 3.0f / 1000.0f;
    float V = voltageDivider * Vo;

    return {T, light.lux, A, V, light.ir};
}

void dhcpLoop() {
    switch (Ethernet.maintain()) {
    case DHCP_CHECK_RENEW_FAIL:
        // renewed fail
        Serial.println(F("ERR: renewed fail"));
        break;

    case DHCP_CHECK_RENEW_OK:
        // renewed success
        Serial.println(F("INF: Renewed success"));
        // print your local IP address:
        Serial.print(F("INF: My IP address "));
        Serial.println(Ethernet.localIP());
        break;

    case DHCP_CHECK_REBIND_FAIL:
        // rebind fail
        Serial.println(F("ERR: rebind fail"));
        break;

    case DHCP_CHECK_REBIND_OK:
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
