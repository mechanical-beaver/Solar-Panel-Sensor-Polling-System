/*
 * Author: @github.com/annadostoevskaya
 * Filename: OPT4003Q1.cpp
 * Created: 01 Jul 2025 07:03:12
 * Last Update: 05 Jul 2025 14:46:46
 *
 * Description: <EMPTY>
 */

#include "OPT4003Q1.h"
#include <string.h>

OPT4003Q1::OPT4003Q1(int32_t sensorID)
    : _initialized(false), _sensorID(sensorID) {}

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

void OPT4003Q1::getSensor(sensor_t *sensor) {
    if (!_initialized) return;

    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "OPT4003Q1", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_LIGHT;
    sensor->min_delay = 0;
    sensor->max_value = 143000.0;
    sensor->min_value = 0.0;
    sensor->resolution = 1e-6;
}

float OPT4003Q1::getALS() {
    OPT4003Q1_ResultHigh rh = {};
    OPT4003Q1_ResultLow rl = {};
    rh.raw = readx(OPT4003Q1_REGISTER_CH0_RESULT_HIGH);
    rl.raw = readx(OPT4003Q1_REGISTER_CH0_RESULT_LOW);

    uint32_t r = ((rh.msb << 8) + rl.lsb) << rh.e;

    // TODO: CRC Check

    return (double)r * 535e-6;
}

double OPT4003Q1::getIR() {
    OPT4003Q1_ResultHigh rh = {};
    OPT4003Q1_ResultLow rl = {};
    rh.raw = readx(OPT4003Q1_REGISTER_CH1_RESULT_HIGH);
    rl.raw = readx(OPT4003Q1_REGISTER_CH1_RESULT_LOW);

    uint32_t r = ((rh.msb << 8) + rl.lsb) << rh.e;

    // TODO: CRC Check

    return (double)r * 409e-12;
}

bool OPT4003Q1::getEvent(sensors_event_t *e) {
    /*uint16_t ir, full;*/
    /*uint32_t lum = getFullLuminosity();*/
    /* Early silicon seems to have issues when there is a sudden jump in */
    /* light levels. :( To work around this for now sample the sensor 2x */
    /*lum = getFullLuminosity();*/
    /*ir = lum >> 16;*/
    /*full = lum & 0xFFFF;*/

    /* Clear the event */
    memset(e, 0, sizeof(sensors_event_t));

    /*e->version = sizeof(sensors_event_t);*/
    /*e->sensor_id = _sensorID;*/
    /*e->type = SENSOR_TYPE_LIGHT;*/
    /*e->timestamp = millis();*/

    /* Calculate the actual lux value */
    /* 0 = sensor overflow (too much light) */
    /*e->light = calculateLux(full, ir);*/

    return true;
}
