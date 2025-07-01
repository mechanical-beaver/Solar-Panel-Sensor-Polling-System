/*
 * Author: @github.com/annadostoevskaya
 * Filename: OPT4003Q1.h
 * Created: 01 Jul 2025 07:03:10
 * Last Update: 01 Jul 2025 14:49:35
 *
 * Description: <EMPTY>
 */

#ifndef _OPT4003Q1_H_
#define _OPT4003Q1_H_

#ifndef _GLIBCXX_TYPE_TRAITS
#include "avr_type_traits.h"
#endif // _GLIBCXX_TYPE_TRAITS

#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

#define OPT4003Q1_I2C_ADDR  (0x44)

#define OPT4003Q1_DEVICE_ID (0x221)

/// OPT4003Q1 I2C address options (based on ADDR pin configuration)
enum {
    OPT4003Q1_I2C_ADDR_GND = 0x44, // ADDR pin connected to GND
    OPT4003Q1_I2C_ADDR_VDD = 0x45, // ADDR pin connected to VDD
    OPT4003Q1_I2C_ADDR_SDA = 0x46, // ADDR pin connected to SDA
    OPT4003Q1_I2C_ADDR_SCL = 0x47, // ADDR pin connected to SCL
};

/// OPT4003Q1 Register map
enum {
    // Channel 0 - Result upper (Exponent + MSB)
    OPT4003Q1_REGISTER_CH0_RESULT_UPPER = 0x00, // EXPONENT + MSB
    OPT4003Q1_REGISTER_CH0_RESULT_EXPONENT = 0x00,
    OPT4003Q1_REGISTER_CH0_RESULT_MSB = 0x00,

    // Channel 0 - Result lower (LSB + Counter + CRC)
    OPT4003Q1_REGISTER_CH0_RESULT_LOWER = 0x01,
    OPT4003Q1_REGISTER_CH0_RESULT_LSB = 0x01,
    OPT4003Q1_REGISTER_CH0_COUNTER = 0x01,
    OPT4003Q1_REGISTER_CH0_CRC = 0x01,

    // Channel 1 - Result upper (Exponent + MSB)
    OPT4003Q1_REGISTER_CH1_RESULT_UPPER = 0x02,
    OPT4003Q1_REGISTER_CH1_RESULT_EXPONENT = 0x02,
    OPT4003Q1_REGISTER_CH1_RESULT_MSB = 0x02,

    // Channel 1 - Result lower (LSB + Counter + CRC)
    OPT4003Q1_REGISTER_CH1_RESULT_LOWER = 0x03,
    OPT4003Q1_REGISTER_CH1_RESULT_LSB = 0x03,
    OPT4003Q1_REGISTER_CH1_COUNTER = 0x03,
    OPT4003Q1_REGISTER_CH1_CRC = 0x03,

    // FIFO CH0
    OPT4003Q1_REGISTER_FIFO_CH0_RESULT_UPPER = 0x04,
    OPT4003Q1_REGISTER_FIFO_CH0_EXPONENT = 0x04,
    OPT4003Q1_REGISTER_FIFO_CH0_RESULT_MSB = 0x04,

    OPT4003Q1_REGISTER_FIFO_CH0_RESULT_LOWER = 0x05,
    OPT4003Q1_REGISTER_FIFO_CH0_RESULT_LSB = 0x05,
    OPT4003Q1_REGISTER_FIFO_CH0_COUNTER = 0x05,
    OPT4003Q1_REGISTER_FIFO_CH0_CRC = 0x05,

    // FIFO CH1
    OPT4003Q1_REGISTER_FIFO_CH1_RESULT_UPPER = 0x06,
    OPT4003Q1_REGISTER_FIFO_CH1_EXPONENT = 0x06,
    OPT4003Q1_REGISTER_FIFO_CH1_RESULT_MSB = 0x06,

    OPT4003Q1_REGISTER_FIFO_CH1_RESULT_LOWER = 0x07,
    OPT4003Q1_REGISTER_FIFO_CH1_RESULT_LSB = 0x07,
    OPT4003Q1_REGISTER_FIFO_CH1_COUNTER = 0x07,
    OPT4003Q1_REGISTER_FIFO_CH1_CRC = 0x07,

    // Threshold low
    OPT4003Q1_REGISTER_THRESHOLD_LOW = 0x08,
    OPT4003Q1_REGISTER_THRESHOLD_LOW_EXPONENT = 0x08,
    OPT4003Q1_REGISTER_THRESHOLD_LOW_RESULT = 0x08,

    // Threshold high
    OPT4003Q1_REGISTER_THRESHOLD_HIGH = 0x09,
    OPT4003Q1_REGISTER_THRESHOLD_HIGH_EXPONENT = 0x09,
    OPT4003Q1_REGISTER_THRESHOLD_HIGH_RESULT = 0x09,

    // Config A
    OPT4003Q1_REGISTER_CONFIG_A = 0x0A,
    OPT4003Q1_REGISTER_QWAKE = 0x0A,
    OPT4003Q1_REGISTER_RANGE = 0x0A,
    OPT4003Q1_REGISTER_CONVERSION_TIME = 0x0A,
    OPT4003Q1_REGISTER_OPERATING_MODE = 0x0A,
    OPT4003Q1_REGISTER_LATCH = 0x0A,
    OPT4003Q1_REGISTER_INT_POL = 0x0A,
    OPT4003Q1_REGISTER_FAULT_COUNT = 0x0A,

    // Config B
    OPT4003Q1_REGISTER_CONFIG_B = 0x0B,
    OPT4003Q1_REGISTER_THRESHOLD_CHANNEL_SELECT = 0x0B,
    OPT4003Q1_REGISTER_INT_DIRECTION = 0x0B,
    OPT4003Q1_REGISTER_INT_CONFIG = 0x0B,
    OPT4003Q1_REGISTER_I2C_BURST = 0x0B,

    // Flags
    OPT4003Q1_REGISTER_FLAGS = 0x0C,
    OPT4003Q1_REGISTER_OVERLOAD_FLAG = 0x0C,
    OPT4003Q1_REGISTER_CONVERSION_READY_FLAG = 0x0C,
    OPT4003Q1_REGISTER_FLAG_HIGH = 0x0C,
    OPT4003Q1_REGISTER_FLAG_LOW = 0x0C,

    // Device ID
    OPT4003Q1_REGISTER_DEVICE_ID = 0x11,
    OPT4003Q1_REGISTER_DEVICE_ID_LOW = 0x11,
    OPT4003Q1_REGISTER_DEVICE_ID_HIGH = 0x11,
};

class OPT4003Q1 : public Adafruit_Sensor {
public:
    OPT4003Q1(int32_t sensorID = -1);
    ~OPT4003Q1();

    boolean begin(TwoWire *theWire, uint8_t addr = OPT4003Q1_I2C_ADDR);
    boolean begin(uint8_t addr = OPT4003Q1_I2C_ADDR);

    /* Unified Sensor API Functions */
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);

private:
    Adafruit_I2CDevice *_i2c = NULL;

    boolean _initialized;

    int32_t _sensorID;
    uint8_t _addr;

    template <typename T>
    typename enable_if<is_same<T, uint8_t>::value, T>::type readx(uint8_t r) {
        uint8_t v;
        _i2c->write_then_read(&r, sizeof(r), &v, sizeof(v));
        return v;
    }

    template <typename T>
    typename enable_if<is_same<T, uint16_t>::value, T>::type readx(uint8_t r) {
        uint16_t v;
        _i2c->write_then_read(&r, sizeof(r), reinterpret_cast<uint8_t *>(&v),
                              sizeof(v));
        return __builtin_bswap16(v);
    }

    void writex(uint8_t r, uint16_t v) {
        _i2c->write(&r, sizeof(r));
        v = __builtin_bswap16(v);
        _i2c->write(reinterpret_cast<uint8_t *>(&v), sizeof(v));
    }

    void writex(uint8_t r) { _i2c->write(&r, 1); }
};

#endif // _OPT4003Q1_H_
