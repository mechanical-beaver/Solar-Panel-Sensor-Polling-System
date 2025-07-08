/*
 * Author: @github.com/annadostoevskaya
 * Filename: OPT4003Q1.cpp
 * Created: 01 Jul 2025 07:03:12
 * Last Update: 08 Jul 2025 15:21:24
 *
 * Description: <EMPTY>
 */

#include "OPT4003Q1.h"

OPT4003Q1::OPT4003Q1() : _initialized(false) {}

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

    uint32_t r = ((rh.msb << 8) + rl.lsb) << rh.e;

    // TODO: CRC Check

    return (double)r * 535e-6;
}

double OPT4003Q1::getIR() {
    // TODO: Error handling
    if (!_initialized) return 0.0;

    OPT4003Q1_ResultHigh rh = {};
    OPT4003Q1_ResultLow rl = {};
    rh.raw = readx(OPT4003Q1_REGISTER_CH1_RESULT_HIGH);
    rl.raw = readx(OPT4003Q1_REGISTER_CH1_RESULT_LOW);

    uint32_t r = ((rh.msb << 8) + rl.lsb) << rh.e;

    // TODO: CRC Check

    return (double)r * 409e-12;
}

boolean OPT4003Q1::ready() {
    // TODO: Error handling
    if (!_initialized) return 0.0;

    OPT4003Q1_Config cfg = {};
    cfg.raw = readx(OPT4003Q1_REGISTER_CONFIG_A);
    return cfg.operatingMode == OPT4003Q1_POWER_DOWN_MODE;
}
