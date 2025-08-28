/*
 * Author: @github.com/annadostoevskaya
 * Filename: OPT4003Q1.h
 * Created: 01 Jul 2025 07:03:10
 * Last Update: 28 Aug 2025 14:48:15
 *
 * Description: <EMPTY>
 */

#ifndef _OPT4003Q1_H_
#define _OPT4003Q1_H_

#include <Adafruit_I2CDevice.h>
#include <Arduino.h>

#define OPT4003Q1_I2C_ADDR      (0x44)

#define OPT4003Q1_DEVICE_ID     (0x221)

#define OPT4003Q1_QWAKE_ENABLE  (1)
#define OPT4003Q1_QWAKE_DISABLE (0)

#define OPT4003Q1_RANGE_0       (0x00) ///< 561 lux or 0.429 mW/cm^2
#define OPT4003Q1_RANGE_1       (0x01) ///< 1122 lux or 0.858 mW/cm^2
#define OPT4003Q1_RANGE_2       (0x02) ///< 2244 lux or 1.72 mW/cm^2
#define OPT4003Q1_RANGE_3       (0x03) ///< 4488 lux or 3.43 mW/cm^2
#define OPT4003Q1_RANGE_4       (0x04) ///< 8976 lux or 6.86 mW/cm^2
#define OPT4003Q1_RANGE_5       (0x05) ///< 17952 lux or 13.72 mW/cm^2
#define OPT4003Q1_RANGE_6       (0x06) ///< 35903 lux or 27.45 mW/cm^2
#define OPT4003Q1_RANGE_7       (0x07) ///< 71806 lux or 27.45 mW/cm^2
#define OPT4003Q1_RANGE_8       (0x08) ///< 143613 lux or 27.45 mW/cm^2
#define OPT4003Q1_RANGE_AUTO                                                   \
    (0x0C) ///< Determined by automatic full-scale range logic;
           ///< sets channel 0 and channel 1 independently

#define OPT4003Q1_CONVERSION_TIME_0       (0x00) ///< 0.6ms
#define OPT4003Q1_CONVERSION_TIME_1       (0x01) ///< 1ms
#define OPT4003Q1_CONVERSION_TIME_2       (0x02) ///< 1.8ms
#define OPT4003Q1_CONVERSION_TIME_3       (0x03) ///< 3.4ms
#define OPT4003Q1_CONVERSION_TIME_4       (0x04) ///< 6.5ms
#define OPT4003Q1_CONVERSION_TIME_5       (0x05) ///< 12.7ms
#define OPT4003Q1_CONVERSION_TIME_6       (0x06) ///< 25ms
#define OPT4003Q1_CONVERSION_TIME_7       (0x07) ///< 50ms
#define OPT4003Q1_CONVERSION_TIME_8       (0x08) ///< 100ms
#define OPT4003Q1_CONVERSION_TIME_9       (0x09) ///< 200ms
#define OPT4003Q1_CONVERSION_TIME_10      (0x0A) ///< 400ms
#define OPT4003Q1_CONVERSION_TIME_11      (0x0B) ///< 800ms

#define OPT4003Q1_POWER_DOWN_MODE         (0)
#define OPT4003Q1_FORCED_ONESHOT_MODE     (1)
#define OPT4003Q1_ONESHOT_MODE            (2)
#define OPT4003Q1_CONTINUOUS_MODE         (3)

#define OPT4003Q1_LATCHED_MODE            (1)
#define OPT4003Q1_TRANSPARENT_MODE        (0)

#define OPT4003Q1_ACTIVE_LOW              (0)
#define OPT4003Q1_ACTIVE_HIGH             (1)

#define OPT4003Q1_FAULT_COUNT_0           (0)
#define OPT4003Q1_FAULT_COUNT_1           (1)
#define OPT4003Q1_FAULT_COUNT_2           (2)
#define OPT4003Q1_FAULT_COUNT_3           (3)

#define OPT4003Q1_THRESHOLD_SELECT_CH0    (0)
#define OPT4003Q1_THRESHOLD_SELECT_CH1    (1)

#define OPT4003Q1_INT_DIR_INPUT           (0)
#define OPT4003Q1_INT_DIR_OUTPUT          (1)

#define OPT4003Q1_INT_CFG_SMBUS_ALERT     (0)
#define OPT4003Q1_INT_CFG_ONE_CONVERSION  (1)
#define OPT4003Q1_INT_CFG_TWO_CONVERSION  (2)
#define OPT4003Q1_INT_CFG_FOUR_CONVERSION (3)

#define OPT4003Q1_I2C_BURST_ENABLE        (1)
#define OPT4003Q1_I2C_BURST_DISABLE       (0)

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
    OPT4003Q1_REGISTER_CH0_RESULT_HIGH = 0x00, // EXPONENT + MSB
    OPT4003Q1_REGISTER_CH0_RESULT_EXPONENT = 0x00,
    OPT4003Q1_REGISTER_CH0_RESULT_MSB = 0x00,

    // Channel 0 - Result lower (LSB + Counter + CRC)
    OPT4003Q1_REGISTER_CH0_RESULT_LOW = 0x01,
    OPT4003Q1_REGISTER_CH0_RESULT_LSB = 0x01,
    OPT4003Q1_REGISTER_CH0_COUNTER = 0x01,
    OPT4003Q1_REGISTER_CH0_CRC = 0x01,

    // Channel 1 - Result upper (Exponent + MSB)
    OPT4003Q1_REGISTER_CH1_RESULT_HIGH = 0x02,
    OPT4003Q1_REGISTER_CH1_RESULT_EXPONENT = 0x02,
    OPT4003Q1_REGISTER_CH1_RESULT_MSB = 0x02,

    // Channel 1 - Result lower (LSB + Counter + CRC)
    OPT4003Q1_REGISTER_CH1_RESULT_LOW = 0x03,
    OPT4003Q1_REGISTER_CH1_RESULT_LSB = 0x03,
    OPT4003Q1_REGISTER_CH1_COUNTER = 0x03,
    OPT4003Q1_REGISTER_CH1_CRC = 0x03,

    // FIFO CH0
    OPT4003Q1_REGISTER_FIFO_CH0_RESULT_HIGH = 0x04,
    OPT4003Q1_REGISTER_FIFO_CH0_EXPONENT = 0x04,
    OPT4003Q1_REGISTER_FIFO_CH0_RESULT_MSB = 0x04,

    OPT4003Q1_REGISTER_FIFO_CH0_RESULT_LOW = 0x05,
    OPT4003Q1_REGISTER_FIFO_CH0_RESULT_LSB = 0x05,
    OPT4003Q1_REGISTER_FIFO_CH0_COUNTER = 0x05,
    OPT4003Q1_REGISTER_FIFO_CH0_CRC = 0x05,

    // FIFO CH1
    OPT4003Q1_REGISTER_FIFO_CH1_RESULT_HIGH = 0x06,
    OPT4003Q1_REGISTER_FIFO_CH1_EXPONENT = 0x06,
    OPT4003Q1_REGISTER_FIFO_CH1_RESULT_MSB = 0x06,

    OPT4003Q1_REGISTER_FIFO_CH1_RESULT_LOW = 0x07,
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

enum OPT4003Q1_Error {
    OPT4003Q1_ERROR_OK = 0,
    OPT4003Q1_ERROR_I2C_BEGIN_FAILED,
    OPT4003Q1_ERROR_I2C_WRITE_FAILED,
    OPT4003Q1_ERROR_NOT_INITIALIZED,
    OPT4003Q1_ERROR_INVALID_DEVICE_ID,
    OPT4003Q1_ERROR_CRC_FAILED,
};

#pragma pack(push, 1)
union OPT4003Q1_Config {
    struct {
        uint16_t faultCount : 2;
        uint16_t interruptPolarity : 1;
        uint16_t latch : 1;
        uint16_t operatingMode : 2;
        uint16_t conversionTime : 4;
        uint16_t range : 4;
        uint16_t : 1;
        uint16_t qwake : 1;
    };
    uint16_t raw;
};

union OPT4003Q1_IntConfig {
    struct {
        uint16_t i2cBurst : 1;
        uint16_t : 1;
        uint16_t intCfg : 2;
        uint16_t intDir : 1;
        uint16_t threadholdChSel : 1;
        uint16_t : 10;
    };
    uint16_t raw;
};

union OPT4003Q1_Status {
    struct {
        uint16_t flagl : 1;
        uint16_t flagh : 1;
        uint16_t conversionReadyFlag : 1;
        uint16_t overloadFlag : 1;
        uint16_t : 12;
    };
    uint16_t raw;
};

union OPT4003Q1_ResultHigh {
    struct {
        uint16_t msb : 12;
        uint16_t e : 4;
    };
    uint16_t raw;
};

union OPT4003Q1_ResultLow {
    struct {
        uint16_t crc : 4;
        uint16_t counter : 4;
        uint16_t lsb : 8;
    };

    uint16_t raw;
};
#pragma pack(pop)

struct OPT4003Q1_Light {
    double ir;
    float lux;
};

class OPT4003Q1 {
public:
    OPT4003Q1(boolean enableCrc = false);
    ~OPT4003Q1();

    boolean begin(TwoWire *theWire, uint8_t addr = OPT4003Q1_I2C_ADDR);
    boolean begin(uint8_t addr = OPT4003Q1_I2C_ADDR);

    void disable();
    void enable(OPT4003Q1_Config cfg = {
                    {OPT4003Q1_FAULT_COUNT_0, OPT4003Q1_ACTIVE_LOW,
                     OPT4003Q1_LATCHED_MODE, OPT4003Q1_FORCED_ONESHOT_MODE,
                     OPT4003Q1_CONVERSION_TIME_8, OPT4003Q1_RANGE_AUTO,
                     OPT4003Q1_QWAKE_ENABLE}});

    float getALS();
    double getIR();

    boolean isEnable();
    inline boolean isReady() {
        OPT4003Q1_Status cfg = {};
        cfg.raw = readx(OPT4003Q1_REGISTER_FLAGS);
        return cfg.conversionReadyFlag;
    }

    OPT4003Q1_Error getError() { return _errno; }

private:
    Adafruit_I2CDevice *_i2c = NULL;

    boolean _initialized;

    uint8_t _addr;
    boolean _enableCrc;

    OPT4003Q1_Error _errno = OPT4003Q1_ERROR_OK;

    uint16_t readx(uint8_t r);
    bool writex(uint8_t r, uint16_t v);
    boolean verifyCrc(uint32_t m, uint8_t e, uint8_t c, uint8_t crc);
    inline uint8_t parity32(uint32_t v) {
        v ^= v >> 16;
        v ^= v >> 8;
        v ^= v >> 4;
        v ^= v >> 2;
        v ^= v >> 1;
        return (uint8_t)(v & 1u);
    };
};

#endif // _OPT4003Q1_H_
