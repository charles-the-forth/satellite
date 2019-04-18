#include "Arduino.h"

#include "Open_Cansat_GPS.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <MPU9250.h>
#include <RFM69.h>

#define BME280_ADRESS 0x76
#define BME280_ADDRESS_OPEN_CANSAT 0x77
#define SEALEVELPRESSURE_HPA 1013.25
// RFM69
#define NETWORKID       0        // Must be the same for all nodes (0 to 255)
#define MYNODEID        1          // My node ID (0 to 255)
#define TONODEID        2          // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY       RF69_433MHZ   // Frequency set up
#define FREQUENCYSPECIFIC 433102000  // Should be value in Hz, now 433 Mhz will be set
#define CHIP_SELECT_PIN   43 //radio chip select
#define INTERUP_PIN       9 //radio interrupt

const int AIR_QUALITY_SENSOR_PIN = A0;
const int CAPACITIVE_SOIL_MOISTURE_SENSOR_PIN = A1;
const int UV_SENSOR_PIN = A2;
const int AIR_QUALITY_SENSOR_LED_PIN = 3;

const int TIME_OF_MEASUREMENT = 280;
const int TIME_OF_EQUALITY = 40;
const int TIME_OF_SLEEP = 9680;

OpenCansatGPS gps;
Adafruit_BME280 bme;
Adafruit_BME280 bme_cansat;
BH1750 lightMeter;
RFM69 radio(CHIP_SELECT_PIN, INTERUP_PIN, true);

#define Serial SerialUSB

MPU9250 IMU(Wire,0x68);
int status;

typedef struct
{
    uint16_t messageId;
    float temperatureCanSat;
    float pressureCanSat;
    float humidityCanSat;
    uint32_t lightIntensity;
    float altitudeCanSat;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t numberOfSatellites;
    uint8_t latInt;
    uint8_t lonInt;
    uint32_t latAfterDot;
    uint32_t lonAfterDot;
  
} messageOut;

messageOut data;

bool isRadioOk = true;

#define Serial SerialUSB
void setup() {
  Serial.begin(57600);

  gps.begin();
  gps.debugPrintOn(57600);
  
  if (!bme.begin(BME280_ADRESS)) {
    Serial.println("External BME280 not found!");
  }
  
  if (!bme_cansat.begin(BME280_ADDRESS_OPEN_CANSAT))
  {
    Serial.println("Internal BME280 not found!");
  }
  
  lightMeter.begin();

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
  }

  if(!radio.initialize(FREQUENCY, MYNODEID, NETWORKID))
  {
    isRadioOk = false;
    Serial.println("RFM69HW initialization failed!");
  }
  else
  {
    radio.setFrequency(FREQUENCYSPECIFIC);
    radio.setHighPower(true);
    Serial.println("RFM69HW successful!");
  }

  data.messageId = 0;
}

void loop() {
  data.messageId++;

  data.lightIntensity = lightMeter.readLightLevel();
  
  int capacitiveSoilMoistureSensorValue = analogRead(CAPACITIVE_SOIL_MOISTURE_SENSOR_PIN);

  uint16_t uvIndex = measureUVSensor();

  data.temperatureCanSat = bme_cansat.readTemperature();
  float temperatureMPU = IMU.getTemperature_C();
  float temperatureExternal = bme.readTemperature();

  float airQuality = measureAirQuality();

  data.pressureCanSat = bme_cansat.readPressure() / 100.0F;
  float pressureExternal = bme.readPressure() / 100.0F;

  data.humidityCanSat = bme_cansat.readHumidity();
  float humidityExternal = bme.readHumidity();

  data.altitudeCanSat = bme_cansat.readAltitude(SEALEVELPRESSURE_HPA);
  float altitudeExternal = bme.readAltitude(SEALEVELPRESSURE_HPA);

  IMU.readSensor();
  float accelerationX = IMU.getAccelX_mss();
  float accelerationY = IMU.getAccelY_mss();
  float accelerationZ = IMU.getAccelZ_mss();

  float rotationX = IMU.getGyroX_rads() * 180 / PI;
  float rotationY = IMU.getGyroY_rads() * 180 / PI;
  float rotationZ = IMU.getGyroZ_rads() * 180 / PI;

  float magnetometerX = IMU.getMagX_uT();
  float magnetometerY = IMU.getMagY_uT();
  float magnetometerZ = IMU.getMagZ_uT();

  if(isRadioOk)
  {
    //Serial.println("Signal = " + static_cast<String>(radio.RSSI));

    Serial.println("Altitude = " + String(data.altitudeCanSat));

    radio.send(TONODEID, (const void*)&data, sizeof(data));
  }
  delay(350);
}

void scanGPS() {
  gps.scan(350);
}

float measureAirQuality() {
  digitalWrite(AIR_QUALITY_SENSOR_LED_PIN, LOW);
  delayMicroseconds(TIME_OF_MEASUREMENT);
  float measuredVoltage = analogRead(AIR_QUALITY_SENSOR_LED_PIN);
  delayMicroseconds(TIME_OF_EQUALITY);
  digitalWrite(AIR_QUALITY_SENSOR_LED_PIN, HIGH);
  delayMicroseconds(TIME_OF_SLEEP);
  return measuredVoltage * (3.3 / 1024.0);
}


float measureUVSensor() {
  float uvSensorValue = 0;
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
    return uvSensorValue += (1.0 / indexDiff) * valueDiff - 1.0;
  }

  return 0;
}
