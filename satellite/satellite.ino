#include "Arduino.h"

#include <Adafruit_INA219.h>
#include <SD.h>
#include "Open_Cansat_GPS.h"
#include <BH1750.h>

#include <RFM69.h>
#include <SPI.h>

#define Serial SerialUSB

const int VIBRATION_PIN = A0;
const int AIR_QUALITY_SENSOR_PIN = A1;
const int CAPACITIVE_SOIL_MOISTURE_SENSOR_PIN = A2;
const int UV_SENSOR_PIN = A3;
const int AIR_QUALITY_SENSOR_LED_PIN = 3;

const int TIME_OF_MEASUREMENT = 280;
const int TIME_OF_EQUALITY = 40;
const int TIME_OF_SLEEP = 9680;

float airQualityValue = 0;
int vibrationSensorValue = 0;
int capacitiveSoilMoistureSensorValue = 0;
float uvSensorValue = 0;
int lightIntesity = 0;

OpenCansatGPS gps;
BH1750 lightMeter;

void setup() {
  Serial.begin(57600); 

  //TODO - check if GPS connected

  gps.begin();

  //gps.debugPrintOn(57600);

  lightMeter.begin();

  pinMode(VIBRATION_PIN, INPUT);
  pinMode(AIR_QUALITY_SENSOR_LED_PIN, OUTPUT);
}

void loop() {
  measureAirQuality();
  measureVibrations();
  measureCapacitiveSoilMoistureSensor();
  measureUVSensor();
  measureLightIntensity();

  gps.scan(350);
  Serial.println(String(airQualityValue) + ";" + String(vibrationSensorValue) + ";" + gps.getLat() + ";" + gps.getLon() + ";" + String(gps.getNumberOfSatellites()) + ";" + String(gps.getYear()) + ";" + String(gps.getMonth()) + ";" + String(gps.getDay()) + ";" + String(gps.getHour()) + ";" + String(gps.getMinute()) + ";" + String(gps.getSecond()) + ";" + String(capacitiveSoilMoistureSensorValue) + ";" + String(uvSensorValue) + ";" + String(lightIntesity));
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

void measureCapacitiveSoilMoistureSensor() {
  capacitiveSoilMoistureSensorValue = analogRead(CAPACITIVE_SOIL_MOISTURE_SENSOR_PIN);
}

void measureUVSensor() {
  int uvAnalog = analogRead(UV_SENSOR_PIN);
  float uvVoltage = uvAnalog * (3300.0 / 1024.0);
  int uvIndexLimits [12] = { 50, 227, 318, 408, 503, 606, 696, 795, 881, 976, 1079, 1170};
  int i;

  //Max measurable value.
  if (uvVoltage > 1170) {
    uvVoltage = 1170;
  }
  
  for (i = 0; i < 12; i++) {
    if (uvAnalog <= uvIndexLimits[i]) {
      uvSensorValue = i;
      break;
    }
  }

  if (i > 0) {
    float indexDiff = uvIndexLimits[i] - uvIndexLimits[i - 1];
    float valueDiff = uvAnalog - uvIndexLimits[i - 1];
    uvSensorValue += (1.0 / indexDiff) * valueDiff - 1.0;
  }
}

void measureLightIntensity() {
  lightIntesity = lightMeter.readLightLevel();
}
