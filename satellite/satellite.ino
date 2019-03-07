#include "Arduino.h"

#include <Adafruit_INA219.h>
#include <SD.h>
#include "Open_Cansat_GPS.h"
#include <BH1750.h>

#include <RFM69.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME280_ADDRESS_OPEN_CANSAT 0x77
#define SEALEVELPRESSURE_HPA 1013.25

#define Serial SerialUSB

const int AIR_QUALITY_SENSOR_PIN = A0;
const int CAPACITIVE_SOIL_MOISTURE_SENSOR_PIN = A1;
const int UV_SENSOR_PIN = A2;
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
Adafruit_BME280 bme_cansat;

void setup() {
  Serial.begin(57600); 

  //TODO - check if GPS connected

  gps.begin();

  if (!bme_cansat.begin(BME280_ADDRESS_OPEN_CANSAT))
  {
    Serial.println("CanSat BME280 sensor not found!");
  }

  //gps.debugPrintOn(57600);

  lightMeter.begin();

  pinMode(AIR_QUALITY_SENSOR_LED_PIN, OUTPUT);
}

void loop() {
  measureAirQuality();
  measureCapacitiveSoilMoistureSensor();
  measureUVSensor();
  measureLightIntensity();

  gps.scan(350);
  Serial.println(
    String(airQualityValue) + ";" + gps.getLat() + ";" + gps.getLon() + ";" + String(gps.getNumberOfSatellites()) + ";" 
    + String(gps.getYear()) + ";" + String(gps.getMonth()) + ";" + String(gps.getDay()) + ";" + String(gps.getHour()) + ";" + String(gps.getMinute()) + ";" + String(gps.getSecond()) + ";" 
    + String(capacitiveSoilMoistureSensorValue) + ";" + String(uvSensorValue) + ";" + String(lightIntesity) + ";" + String(bme_cansat.readTemperature()) + ";" + String(bme_cansat.readHumidity()) + ";" 
    + String(bme_cansat.readPressure() / 100.0F) + ";" + String((bme_cansat.readAltitude(SEALEVELPRESSURE_HPA)))
  );
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
