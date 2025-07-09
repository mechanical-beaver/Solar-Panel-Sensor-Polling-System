/*
 * Author: @github.com/annadostoevskaya
 * Filename: OPT4003Q1.cpp
 * Created: 01 Jul 2025 07:03:12
 * Last Update: 09 Jul 2025 15:29:06
 *
 * Description: <EMPTY>
 */

#include "OPT4003Q1.h"
#include "Arduino.h"

OPT4003Q1::OPT4003Q1(boolean enableCrc)
    : _initialized(false), _enableCrc(enableCrc) {}

OPT4003Q1::~OPT4003Q1() {
    // TODO: Error handling
    if (_i2c) delete _i2c;
}

boolean OPT4003Q1::begin(TwoWire *theWire, uint8_t addr) {
    // TODO: Error handling
    if (_i2c) delete _i2c;

    _i2c = new Adafruit_I2CDevice(addr, theWire);
    if (!_i2c->begin()) return false;

    uint16_t id = readx(OPT4003Q1_REGISTER_DEVICE_ID);

    if (id != OPT4003Q1_DEVICE_ID) return false;

    _initialized = true;

    disable();

    return true;
}

boolean OPT4003Q1::begin(uint8_t addr) { return begin(&Wire, addr); }

void OPT4003Q1::disable() {
    // TODO: Error handling
    if (!_initialized) return;

    OPT4003Q1_Config cfg = {};
    cfg.operatingMode = OPT4003Q1_POWER_DOWN_MODE;
    cfg.qwake = OPT4003Q1_QWAKE_ENABLE;

    writex(OPT4003Q1_REGISTER_CONFIG_A, cfg.raw);
}

void OPT4003Q1::enable() {
    // TODO: Error handling
    if (!_initialized) return;

    OPT4003Q1_Config cfg = {{OPT4003Q1_FAULT_COUNT_0, OPT4003Q1_ACTIVE_LOW,
                             OPT4003Q1_LATCHED_MODE, OPT4003Q1_ONESHOT_MODE,
                             OPT4003Q1_CONVERSION_TIME_8, OPT4003Q1_RANGE_AUTO,
                             OPT4003Q1_QWAKE_ENABLE}};

    writex(OPT4003Q1_REGISTER_CONFIG_A, cfg.raw);
}

float OPT4003Q1::getALS() {
    // TODO: Error handling
    if (!_initialized) return 0.0f;

    OPT4003Q1_ResultHigh rh = {};
    OPT4003Q1_ResultLow rl = {};
    rh.raw = readx(OPT4003Q1_REGISTER_CH0_RESULT_HIGH);
    rl.raw = readx(OPT4003Q1_REGISTER_CH0_RESULT_LOW);

    uint32_t m = (uint32_t)(rh.msb << 8) + rl.lsb;
    uint32_t r = m << rh.e;

    if (_enableCrc && !verifyCrc(m, rh.e, rl.counter, rl.crc)) {
#ifdef _OPT4003Q1_VERBOSE_
        Serial.println("[err] crc check failed!");
#endif
        r = 0;
    }

    return (double)r * 535e-6;
}

double OPT4003Q1::getIR() {
    // TODO: Error handling
    if (!_initialized) return 0.0;

    OPT4003Q1_ResultHigh rh = {};
    OPT4003Q1_ResultLow rl = {};
    rh.raw = readx(OPT4003Q1_REGISTER_CH1_RESULT_HIGH);
    rl.raw = readx(OPT4003Q1_REGISTER_CH1_RESULT_LOW);

    uint32_t m = (uint32_t)(rh.msb << 8) + rl.lsb;
    uint32_t r = m << rh.e;

    if (_enableCrc && !verifyCrc(m, rh.e, rl.counter, rl.crc)) {
#ifdef _OPT4003Q1_CRC_VERBOSE_
        Serial.println("[err] crc check failed!");
#endif
        r = 0;
    }

    return (double)r * 409e-12;
}

boolean OPT4003Q1::isEnable() {
    // TODO: Error handling
    if (!_initialized) return 0.0;

    OPT4003Q1_Config cfg = {};
    cfg.raw = readx(OPT4003Q1_REGISTER_CONFIG_A);
    return cfg.operatingMode != OPT4003Q1_POWER_DOWN_MODE;
}

boolean OPT4003Q1::verifyCrc(uint32_t m, uint8_t e, uint8_t c, uint8_t crc) {
    uint8_t x[4] = {0};

    // X[0] = XOR(E[3:0], R[19:0], C[3:0])
    for (int i = 0; i < 4; ++i)
        x[0] ^= (e >> i) & 1;

    for (int i = 0; i < 20; ++i)
        x[0] ^= (m >> i) & 1;

    for (int i = 0; i < 4; ++i)
        x[0] ^= (c >> i) & 1;

    // X[1] = XOR(C[1], C[3], R[1], R[3], ..., R[19], E[1], E[3])
    x[1] ^= (c >> 1) & 1;
    x[1] ^= (c >> 3) & 1;

    for (int i = 1; i <= 19; i += 2)
        x[1] ^= (m >> i) & 1;

    x[1] ^= (e >> 1) & 1;
    x[1] ^= (e >> 3) & 1;

    // X[2] = XOR(C[3], R[3], R[7], R[11], R[15], R[19], E[3])
    x[2] ^= (c >> 3) & 1;
    x[2] ^= (m >> 3) & 1;
    x[2] ^= (m >> 7) & 1;
    x[2] ^= (m >> 11) & 1;
    x[2] ^= (m >> 15) & 1;
    x[2] ^= (m >> 19) & 1;
    x[2] ^= (e >> 3) & 1;

    // X[3] = XOR(R[3], R[11], R[19])
    x[3] ^= (m >> 3) & 1;
    x[3] ^= (m >> 11) & 1;
    x[3] ^= (m >> 19) & 1;

    uint8_t computedCrc =
        (uint8_t)((x[3] << 3) | (x[2] << 2) | (x[1] << 1) | x[0]);

#ifdef _OPT4003Q1_CRC_VERBOSE_
    Serial.print("[inf] crc: ");
    Serial.println(crc, BIN);
    Serial.print("[inf] computed crc: ");
    Serial.println(computedCrc, BIN);
#endif

    return (computedCrc == (crc & 0x0F));
}
