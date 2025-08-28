/*
 * Author: @github.com/annadostoevskaya
 * Filename: OPT4003Q1.cpp
 * Created: 01 Jul 2025 07:03:12
 * Last Update: 28 Aug 2025 14:28:36
 *
 * Description: <EMPTY>
 */

#include "OPT4003Q1.h"

OPT4003Q1::OPT4003Q1(boolean enableCrc)
    : _initialized(false), _enableCrc(enableCrc) {}

OPT4003Q1::~OPT4003Q1() {
    if (_i2c) delete _i2c;
}

boolean OPT4003Q1::begin(TwoWire *theWire, uint8_t addr) {
    if (_i2c) delete _i2c;

    _i2c = new Adafruit_I2CDevice(addr, theWire);
    if (!_i2c->begin()) {
        _errno = OPT4003Q1_ERROR_I2C_BEGIN_FAILED;
        return false;
    }

    uint16_t id = readx(OPT4003Q1_REGISTER_DEVICE_ID);
    if (id != OPT4003Q1_DEVICE_ID) {
        _errno = OPT4003Q1_ERROR_INVALID_DEVICE_ID;
        return false;
    }

    _initialized = true;

    disable();

    return true;
}

boolean OPT4003Q1::begin(uint8_t addr) { return begin(&Wire, addr); }

void OPT4003Q1::enable(OPT4003Q1_Config cfg) {
    if (!_initialized) {
        _errno = OPT4003Q1_ERROR_NOT_INITIALIZED;
        return;
    }

    if (writex(OPT4003Q1_REGISTER_CONFIG_A, cfg.raw)) {
        _errno = OPT4003Q1_ERROR_I2C_WRITE_FAILED;
        return;
    }

    isReady();
}

void OPT4003Q1::disable() {
    if (!_initialized) {
        _errno = OPT4003Q1_ERROR_NOT_INITIALIZED;
        return;
    }

    OPT4003Q1_Config cfg = {};
    cfg.operatingMode = OPT4003Q1_POWER_DOWN_MODE;
    cfg.qwake = OPT4003Q1_QWAKE_ENABLE;

    if (writex(OPT4003Q1_REGISTER_CONFIG_A, cfg.raw)) {
        _errno = OPT4003Q1_ERROR_I2C_WRITE_FAILED;
        return;
    }
}

float OPT4003Q1::getALS() {
    if (!_initialized) {
        _errno = OPT4003Q1_ERROR_NOT_INITIALIZED;
        return 0.0f;
    }

    OPT4003Q1_ResultHigh rh = {};
    OPT4003Q1_ResultLow rl = {};
    rh.raw = readx(OPT4003Q1_REGISTER_CH0_RESULT_HIGH);
    rl.raw = readx(OPT4003Q1_REGISTER_CH0_RESULT_LOW);

    uint32_t m = ((uint32_t)rh.msb << 8) | rl.lsb;
    uint32_t r = m * (1UL << rh.e);

    if (_enableCrc && !verifyCrc(m, rh.e, rl.counter, rl.crc)) {
        _errno = OPT4003Q1_ERROR_CRC_FAILED;
        return 0.0f;
    }

    return (double)r * 535e-6;
}

double OPT4003Q1::getIR() {
    if (!_initialized) {
        _errno = OPT4003Q1_ERROR_NOT_INITIALIZED;
        return 0.0;
    }

    OPT4003Q1_ResultHigh rh = {};
    OPT4003Q1_ResultLow rl = {};
    rh.raw = readx(OPT4003Q1_REGISTER_CH1_RESULT_HIGH);
    rl.raw = readx(OPT4003Q1_REGISTER_CH1_RESULT_LOW);

    uint32_t m = ((uint32_t)rh.msb << 8) | rl.lsb;
    uint32_t r = m * (1UL << rh.e);

    if (_enableCrc && !verifyCrc(m, rh.e, rl.counter, rl.crc)) {
        _errno = OPT4003Q1_ERROR_CRC_FAILED;
        return 0.0;
    }

    return (double)r * 409e-12;
}

boolean OPT4003Q1::isEnable() {
    if (!_initialized) {
        _errno = OPT4003Q1_ERROR_NOT_INITIALIZED;
        return false;
    }

    OPT4003Q1_Config cfg = {};
    cfg.raw = readx(OPT4003Q1_REGISTER_CONFIG_A);
    return cfg.operatingMode != OPT4003Q1_POWER_DOWN_MODE;
}

uint16_t OPT4003Q1::readx(uint8_t r) {
    uint16_t v = 0;
    _i2c->write_then_read(&r, sizeof(r), reinterpret_cast<uint8_t *>(&v),
                          sizeof(v));
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
    v = __builtin_bswap16(v);
#endif
    return v;
}

bool OPT4003Q1::writex(uint8_t r, uint16_t v) {
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
    v = __builtin_bswap16(v);
#endif
    uint8_t b[] = {r, (uint8_t)(v >> 8), (uint8_t)(v & 0xFF)};
    return _i2c->write(b, sizeof(b));
}

static inline uint8_t parity32(uint32_t v) {
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8_t)(v & 1u);
}

boolean OPT4003Q1::verifyCrc(uint32_t m, uint8_t e, uint8_t c, uint8_t crc) {
    constexpr uint32_t M_ALL = 0xFFFFFu;
    constexpr uint32_t M_ODD = 0xAAAAAu;
    constexpr uint32_t M_X2 = 0x88888u;
    constexpr uint32_t M_X3 = 0x80808u;

    constexpr uint8_t C_X1 = 0x0A;
    constexpr uint8_t C_X2 = 0x08;
    constexpr uint8_t E_X1 = 0x0A;
    constexpr uint8_t E_X2 = 0x08;

    m &= M_ALL;
    e &= 0x0F;
    c &= 0x0F;
    crc &= 0x0F;

    // x0 = parity(E[3:0] ^ R[19:0] ^ C[3:0])
    uint8_t x0 = parity32((uint32_t)(e & 0x0F)) ^ parity32(m) ^
                 parity32((uint32_t)(c & 0x0F));

    // x1 = parity(C[1], C[3], R[1],R[3],...,R[19], E[1],E[3])
    uint8_t x1 = parity32((uint32_t)(c & C_X1)) ^ parity32(m & M_ODD) ^
                 parity32((uint32_t)(e & E_X1));

    // x2 = parity(C[3], R[3], R[7], R[11], R[15], R[19], E[3])
    uint8_t x2 = parity32((uint32_t)(c & C_X2)) ^ parity32(m & M_X2) ^
                 parity32((uint32_t)(e & E_X2));

    // x3 = parity(R[3], R[11], R[19])
    uint8_t x3 = parity32(m & M_X3);

    uint8_t computed = (uint8_t)((x3 << 3) | (x2 << 2) | (x1 << 1) | x0);
    return (computed & 0x0F) == (crc & 0x0F);
}
