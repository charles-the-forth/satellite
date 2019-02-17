#include "Arduino.h"

#include <Adafruit_BME280.h>  // include Adafruit BME280 library
#include <Adafruit_INA219.h>  // include INA219
#include <SD.h>          // include Arduino SD library
#include "Open_Cansat_GPS.h"

#include <RFM69.h>       // include RFM69 library
#include <SPI.h>


#define Serial SerialUSB

const int VIBRATION_PIN = A0;
const int AIR_QUALITY_SENSOR_PIN = A1;
const int AIR_QUALITY_SENSOR_LED_PIN = 3;

const int TIME_OF_MEASUREMENT = 280;
const int TIME_OF_EQUALITY = 40;
const int TIME_OF_SLEEP = 9680;

float airQualityValue = 0;
int vibrationSensorValue = 0;

OpenCansatGPS gps;

void setup() {
  Serial.begin(57600); 

  Serial.println("Setup");

  //TODO - check if GPS connected

  gps.begin();

  //gps.debugPrintOn(57600);

  pinMode(VIBRATION_PIN, INPUT);
  pinMode(AIR_QUALITY_SENSOR_LED_PIN, OUTPUT);
}

void loop() {
  measureAirQuality();
  measureVibrations();

  gps.scan(350);
  Serial.println(String(airQualityValue) + ";" + String(vibrationSensorValue) + ";" + gps.getLat() + ";" + gps.getLon() + ";" + String(gps.getNumberOfSatellites()) + ";" + String(gps.getYear()) + ";" + String(gps.getMonth()) + ";" + String(gps.getDay()) + ";" + String(gps.getHour()) + ";" + String(gps.getMinute()) + ";" + String(gps.getSecond()));
  delay(10);
}

void measureAirQuality() {
  digitalWrite(AIR_QUALITY_SENSOR_LED_PIN, LOW);
  delayMicroseconds(TIME_OF_MEASUREMENT);
  float measuredVoltage = analogRead(AIR_QUALITY_SENSOR_LED_PIN);
  delayMicroseconds(TIME_OF_EQUALITY);
  digitalWrite(AIR_QUALITY_SENSOR_LED_PIN, HIGH);
  delayMicroseconds(TIME_OF_SLEEP);
  float voltageConversion = measuredVoltage * (3.3 / 1024.0);
  airQualityValue = (0.17 * voltageConversion - 0.1) * 1000;
}

void measureVibrations() {
  vibrationSensorValue = analogRead(VIBRATION_PIN);
}
