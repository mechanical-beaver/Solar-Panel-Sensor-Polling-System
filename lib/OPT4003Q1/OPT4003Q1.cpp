/*
 * Author: @github.com/annadostoevskaya
 * Filename: OPT4003Q1.cpp
 * Created: 01 Jul 2025 07:03:12
 * Last Update: 02 Jul 2025 02:22:19
 *
 * Description: <EMPTY>
 */

#include "OPT4003Q1.h"
#include <string.h>

OPT4003Q1::OPT4003Q1(int32_t sensorID)
    : _initialized(false), _sensorID(sensorID) {}

OPT4003Q1::~OPT4003Q1() {
    if (_i2c) delete _i2c;
}

boolean OPT4003Q1::begin(TwoWire *theWire, uint8_t addr) {
    if (_i2c) delete _i2c;

    _i2c = new Adafruit_I2CDevice(addr, theWire);
    if (!_i2c->begin()) return false;

    uint16_t id = readx<uint16_t>(OPT4003Q1_REGISTER_DEVICE_ID);

    if (id != OPT4003Q1_DEVICE_ID) return false;

    _initialized = true;

    // Set Default state for OPT4003Q1
    // TODO: disable();

    return true;
}

boolean OPT4003Q1::begin(uint8_t addr) { return begin(&Wire, addr); }

void OPT4003Q1::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    strncpy(sensor->name, "OPT4003Q1", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_LIGHT;
    sensor->min_delay = 0;
    sensor->max_value = 143000.0;
    sensor->min_value = 0.0;
    sensor->resolution = 0.000001;
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
