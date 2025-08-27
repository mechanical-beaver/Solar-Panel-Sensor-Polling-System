// #include <cstdio.h>
#include "ArduinoJson/Array/JsonArray.hpp"
#include "ArduinoJson/Json/JsonSerializer.hpp"
// #include <cstdint>
#include <stdint.h>
// #include <stdio.h>
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
#include <SD.h>

#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ArduinoJson.h>
#include <DHT_U.h>
#include <string.h>
// Lux defs
#define OPT_ADDR           0x45 // Адрес устройства
#define AH_REG_ADDR        0x0A // Адрес регистра концигурации
#define REG_RESULT_CH0_MSB 0x00 // CH0
#define REG_RESULT_CH0_LSB 0x01 // CH0
#define REG_RESULT_CH1_MSB 0x02 // CH1
#define REG_RESULT_CH1_LSB 0x03 // CH1

#define CONFIG_FILENAME    ("CONFIG.TXT")
#define SD_PIN             (4)
#define ACS712_PIN         (A0)
#define RELAY_PIN          10
#define ONE_WIRE_PIN       2

#define VAC_STEP           2
#define VAC_POINTS         25 * VAC_STEP
#define VAC_LAG            500

// #define VAC_POINTS         50

#define R1                 150000.0f
#define R2                 14960.0f

struct meas {
    // float T, H, L, A, V;
    float T, L, mW, A, V;
};

struct command_pack {
    float I[VAC_POINTS];
    float V[VAC_POINTS];
    // uint16_t YY;
    // uint8_t MM;
    // uint8_t DD;
    // uint8_t HH;
    // uint8_t MM;
    // uint8_t SS;
    // uint8_t MSMS;
    int16_t sp_number;
};

const uint8_t pwmPin1 = 3; // OC2B — Timer2
const uint8_t pwmPin2 = 9; // OC1A — Timer1

uint32_t start = 0;
uint32_t dt = 0;
char *mqttbuf;

// Code configuration regitr
const uint16_t AH_REG_CONFIG = 0x32B4;

constexpr float voltageDivider = (R1 + R2) / R2;

// Voltmeter
Adafruit_ADS1015 ads1015;

// Temp init
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {0x00, 0xb0, 0x5a, 0x85, 0x6b, 0x00};

JsonDocument config;

// Networking
EthernetClient net = {};
MQTTClient client;

void getConfig();

// Lux init func
void lux_power_init(int addr, int addr_reg, int conf);
// Lux get data
void lux_pow_data(float *Lx, float *Pw);
// Temp get data
float getTemp();

inline bool mqttconn() {
    const char *username = config["mqtt_username"];
    const char *password = config["mqtt_password"];
    bool succes = client.connect("", username, password);

    client.subscribe("/device/commands");

    return succes;
}

void Timers_PWM_Init();
meas getMeas();
void get_out_pack(struct command_pack &package);
void dhcpLoop();

// MessageHandler
void CommandHandler(String &topic, String &payload) {
    char msg[100];
    sprintf(msg, "Topic: %s; Command: $s", topic.c_str(), payload.c_str());

    JsonDocument request;
    JsonDocument resultJson;

    DeserializationError err = deserializeJson(request, payload.c_str());

    if (!request["sp_number"].is<uint16_t>()) {
        Serial.println("ERR: Dont correct sp_number");
        return;
    }

    String cmd = request["command"];

    if (topic == "/device/commands") {
        if (config["sp_number"] == request["sp_number"]) {
            if (cmd == "start") {
                Serial.println("1");

                command_pack res;

                get_out_pack(res);

                JsonArray I = resultJson["I"].to<JsonArray>();
                for (int i = 0; i < VAC_POINTS; i++) {
                    I.add(res.I[i]);
                }

                JsonArray V = resultJson["V"].to<JsonArray>();
                for (int i = 0; i < VAC_POINTS; i++) {
                    V.add(res.V[i]);
                }

                resultJson["sp_number"] = request["sp_number"];
                String output;
                serializeJson(resultJson, output);
            }
        }
    }
}

void setup() {
    Serial.begin(9600);

    if (!SD.begin(SD_PIN)) {
        Serial.println(F("ERR: SD card initialization failed!"));
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

    lux_power_init(OPT_ADDR, AH_REG_ADDR, AH_REG_CONFIG);
    sensors.begin();

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

            // no point in carrying on, so do nothing forevermore:
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

    client.onMessage(CommandHandler);

    while (!mqttconn()) {
        Serial.print(F("."));
        delay(1000);
    }

    Serial.println("\n INF: MQTT connected ");

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    pinMode(pwmPin1, OUTPUT);
    pinMode(pwmPin2, OUTPUT);

    Timers_PWM_Init();
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
        payload["mW"] = M.mW;
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
    // DeserializationError err = deserializeJson(config, content);
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
    // Get Lux and mW/cm^2
    float L, mW = 0.000;
    lux_pow_data(&L, &mW);

    float T = getTemp();

    float A = (float(analogRead(ACS712_PIN)) * 0.026f) - 0.02f;

    float Vo = (float)ads1015.readADC_SingleEnded(0) * 3.0f / 1000.0f;
    float V = voltageDivider * Vo;

    return {T, L, mW, A, V};
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

// Инициализация OPT4003
void lux_power_init(int addr, int addr_reg, int conf) {

    Wire.begin();

    // Начинаем общение с датчиком
    Wire.beginTransmission(addr);

    // Передаем адрес регистра
    Wire.write(addr_reg);

    // Передаем данные в регистр
    // Передаем 16-битные данные (разделенные на 2 байта)
    Wire.write(highByte(conf));
    Wire.write(lowByte(conf));

    // Завершаем передачу
    Wire.endTransmission();
}

// Считывание данных с датчика
/********************************************************/
void lux_pow_data(float *Lx, float *Pw) {

    uint16_t reg_ch0_msb = 0;
    uint16_t reg_ch0_lsb = 0;
    uint16_t reg_ch1_msb = 0;
    uint16_t reg_ch1_lsb = 0;

    // Reg00
    Wire.beginTransmission(OPT_ADDR);
    Wire.write(REG_RESULT_CH0_MSB);
    Wire.endTransmission(false);

    Wire.requestFrom(OPT_ADDR, /* (uint8_t) */ 2);
    if (Wire.available() == 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        reg_ch0_msb = ((uint16_t)msb << 8) | lsb;
    } else {
        Serial.println("Ошибка чтения регистра 00");
    }

    // Reg01

    Wire.beginTransmission(OPT_ADDR);
    Wire.write(REG_RESULT_CH0_LSB);
    Wire.endTransmission(false);

    Wire.requestFrom(OPT_ADDR, /* (uint8_t) */ 2);
    if (Wire.available() == 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        reg_ch0_lsb = ((uint16_t)msb << 8) | lsb;
    } else {
        Serial.println("Ошибка чтения регистра 01");
    }

    // Reg02

    Wire.beginTransmission(OPT_ADDR);
    Wire.write(REG_RESULT_CH1_MSB);
    Wire.endTransmission(false);

    Wire.requestFrom(OPT_ADDR, /* (uint8_t) */ 2);
    if (Wire.available() == 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        reg_ch1_msb = ((uint16_t)msb << 8) | lsb;
    } else {
        Serial.println("Ошибка чтения регистра 02");
    }

    // Reg03

    Wire.beginTransmission(OPT_ADDR);
    Wire.write(REG_RESULT_CH1_LSB);
    Wire.endTransmission(false);

    Wire.requestFrom(OPT_ADDR, /* (uint8_t) */ 2);
    if (Wire.available() == 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        reg_ch1_lsb = ((uint16_t)msb << 8) | lsb;
    } else {
        Serial.println("Ошибка чтения регистра 03");
    }

    /******************** Извлечение данных для LUX ********************/

    uint8_t exponentta = (reg_ch0_msb >> 12) & 0xF;
    uint32_t result_msb = reg_ch0_msb & 0x0FFF;      // 12 бит
    uint16_t result_lsb = (reg_ch0_lsb >> 8) & 0xFF; // 8 бит
    uint32_t mantissa = (result_msb << 8) | result_lsb;

    // Делаем ADC_CODE
    uint32_t ADC_CODE = (mantissa << exponentta);

    // Формула из даташита для корпуса USON:
    *Lx = (float)ADC_CODE * 0.000535;

    /******************** Извлечения данных для mW/cm^2 ********************/

    exponentta = (reg_ch1_msb >> 12) & 0xF;
    result_msb = reg_ch1_msb & 0x0FFF;      // 12 бит
    result_lsb = (reg_ch1_lsb >> 8) & 0xFF; // 8 бит
    mantissa = (result_msb << 8) | result_lsb;

    // Делаем ADC_CODE
    ADC_CODE = (mantissa << exponentta);

    // Формула из даташита для корпуса USON:
    *Pw = (float)ADC_CODE * 0.000535;
}

float getTemp() {
    sensors.requestTemperatures();
    float T = sensors.getTempCByIndex(0);
    return T;
}

void Timers_PWM_Init() {
    // --- Настройка Timer2 (пин 3 = OC2B) ---
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2A |= (1 << WGM21) | (1 << WGM20); // Fast PWM (8 bit)
    TCCR2A |= (1 << COM2B1);               // ШИМ на OC2B
    TCCR2B |= (1 << CS21);                 // предделитель 8 → ~31.25 кГц
    OCR2B = 0;                             // начальный duty

    // --- Настройка Timer1 (пин 9 = OC1A) ---
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1A |= (1 << WGM10); // Fast PWM 8-bit
    TCCR1B |= (1 << WGM12);
    TCCR1A |= (1 << COM1A1); // ШИМ на OC1A
    TCCR1B |= (1 << CS11);   // предделитель 8 → ~31.25 кГц
    OCR1A = 0;               // начальный duty
}

void get_out_pack(command_pack &package) {

    uint8_t i = 0;
    uint8_t duty1 = 0;
    uint8_t duty2 = 0;

    digitalWrite(RELAY_PIN, HIGH);

    while (duty1 < VAC_POINTS || duty2 < VAC_POINTS) {

        float A;
        float Vo;
        float V;

        if (duty1 < VAC_POINTS) {
            duty1 += VAC_STEP;
            if (duty1 > VAC_POINTS)
                duty1 = VAC_POINTS;
            OCR2B = duty1;

            delay(VAC_LAG);
            // lux_pow_data(&L, &mW);
            // T = getTemp();
            A = -0.04859 * analogRead(ACS712_PIN) + 24.78;
            Vo = (float)ads1015.readADC_SingleEnded(0) * 3.0f / 1000.0f;
            V = voltageDivider * Vo;

            if (i < VAC_POINTS) {
                package.I[i] = A;
                package.V[i] = V;
                i++;
            }
        }

        if (duty2 < VAC_POINTS) {
            duty2 += VAC_STEP;
            if (duty2 > VAC_POINTS)
                duty2 = VAC_POINTS;
            OCR1A = duty2;

            delay(VAC_LAG);
            // lux_pow_data(&L, &mW);
            // T = getTemp();
            A = -0.04859 * analogRead(ACS712_PIN) + 24.78;
            Vo = (float)ads1015.readADC_SingleEnded(0) * 3.0f / 1000.0f;
            V = voltageDivider * Vo;

            if (i < VAC_POINTS) {
                package.I[i] = A;
                package.V[i] = V;
                i++;
            }
        }
    }

    // --- Надёжное отключение ШИМ ---
    /* TCCR2A &= ~((1 << COM2B1) | (1 << COM2B0));  // отключить OC2B
      TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));  // отключить OC1A

        digitalWrite(pwmPin1, LOW);
          digitalWrite(pwmPin2, LOW);
            digitalWrite(RELAY_PIN, LOW);
              */
    if (duty1 >= VAC_POINTS && duty2 >= VAC_POINTS) {
        OCR2B = 0; // сбросить PWM на 3 пине
        OCR1A = 0; // сбросить PWM на 9 пине
        digitalWrite(RELAY_PIN, LOW);

        // если нужно "начинать заново" при следующем вызове:
        duty1 = 0;
        duty2 = 0;
        i = 0;
    }
}
