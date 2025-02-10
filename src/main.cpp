#include <Arduino.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

#include "Adafruit_ADS1X15.h"
#include "DHT.h"
#include "DHT_U.h"

// #define R1 1600
// #define R2 200

uint16_t R1 = 1496;
uint16_t R2 = 200;

#define DHT_pin 2
#define DHT_type DHT22
#define CurSens_pin A0
#define lag 3000

uint32_t timer = 0;

Adafruit_ADS1015 voltmetr;
DHT_Unified thermometer(DHT_pin, DHT_type);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, 12345);

int16_t voltage_digit;
float V_out;
float V_in;

uint16_t current_digit;
float current;

float temperature;
float humidity;
float light;


void setup()
{
  Serial.begin(9600);

  if(!voltmetr.begin())
  {
    Serial.println("ADS1115 don't init");
    while (1){}
  } 

  if(!tsl.begin())
  {
    Serial.print("TCL2561 don't init");
    while(1){}
  }

  thermometer.begin();
}

void loop()
{
  if (millis() - timer >= lag)
  {
    timer = millis();

    sensors_event_t eventTherm;
    sensors_event_t eventLight;

    thermometer.temperature().getEvent(&eventTherm);
    temperature = eventTherm.temperature;

    thermometer.humidity().getEvent(&eventTherm);
    humidity = eventTherm.relative_humidity;

    tsl.getEvent(&eventLight);
    light = eventLight.light;

    voltage_digit = voltmetr.readADC_SingleEnded(0);
    V_out = (float(voltage_digit) * 3.0F)/1000;
    V_in = (float(R1 + R2)/R2)*V_out;

    current_digit = analogRead(CurSens_pin);
    current = float(voltage_digit) * 0.0195;

    Serial.println("");
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println("°C");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" g/m³");
    Serial.print("Voltage: "); Serial.print(V_in); Serial.println(" V");
    Serial.print("Current: "); Serial.print(current); Serial.println(" A");
    Serial.print("Light: "); Serial.print(light); Serial.println(" lux");
    Serial.println("");
  }
}