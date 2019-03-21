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
#define FREQUENCYSPECIFIC 433000000  // Should be value in Hz, now 433 Mhz will be set
#define CHIP_SELECT_PIN   43 //radio chip select
#define INTERUP_PIN       9 //radio interrupt

const int AIR_QUALITY_SENSOR_PIN = A0;
const int CAPACITIVE_SOIL_MOISTURE_SENSOR_PIN = A1;
const int UV_SENSOR_PIN = A2;
const int AIR_QUALITY_SENSOR_LED_PIN = 3;

const int TIME_OF_MEASUREMENT = 280;
const int TIME_OF_EQUALITY = 40;
const int TIME_OF_SLEEP = 9680;

int vibrationSensorValue = 0;
int capacitiveSoilMoistureSensorValue = 0;
float uvSensorValue = 0;
int lightIntesity = 0;

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
  uint16_t lightIntensity;
  float temperatureCanSat;
  float temperatureExternal;
  float temperatureMPU;
  float pressureCanSat;
  float pressureExternal;
  float humidityCanSat;
  float humidityExternal;
  float accelerationX;
  float accelerationY;
  float accelerationZ;
  float rotationX;
  float rotationY;
  float rotationZ;
} messageOut;

messageOut data;

bool isRadioOk = true;

void setup() {
  Serial.begin(9600);

  gps.begin();
  
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
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
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
}

void loop() {
  IMU.readSensor();

  measureLightIntensity();
  measureTemperature();
  measurePressure();
  measureHumidity();
  measureAcceleration();
  measureRotation();

  Serial.println(
    "air\tlat\tlon\tnum\tyear\tmonth\tday\thour\tmin\tsec\tcap\tuv\tlight\tbme_can_t\tbme_can_l\tbme_can_p\tbme_can_al\tbme_t\tbme_l\tbme_p\tbme_al\tAclX\tAclY\tAclZ\tGyrX\tGyrY\tGyrZ\tMagX\tMagY\tMagZ"
  );
  Serial.println(
    String(capacitiveSoilMoistureSensorValue) + "\t" + String(uvSensorValue) + "\t" + String(lightIntesity) + "\t" + String(bme_cansat.readTemperature()) + "\t\t" + String(bme_cansat.readHumidity()) + "\t\t"
    + String(bme_cansat.readPressure() / 100.0F) + "\t\t" + String((bme_cansat.readAltitude(SEALEVELPRESSURE_HPA))) + "\t\t" + String(bme.readTemperature()) + "\t" + String(bme.readHumidity()) + "\t"
    + String(bme.readPressure() / 100.0F) + "\t" + String(bme.readAltitude(SEALEVELPRESSURE_HPA)) +  "\t" + String(IMU.getAccelX_mss()) + "\t" + String(IMU.getAccelY_mss()) + "\t" + String(IMU.getAccelZ_mss()) + "\t"
    + String((IMU.getGyroX_rads() * 180 / PI), 6) + "\t" + String((IMU.getGyroY_rads() * 180 / PI), 6) + "\t" + String((IMU.getGyroZ_rads() * 180 / PI), 6) + "\t" + String(IMU.getMagX_uT()) + "\t"
    + String(IMU.getMagY_uT()) + "\t" + String(IMU.getMagZ_uT())
  );

  if(isRadioOk)
  {
    Serial.println("Signal = " + static_cast<String>(radio.RSSI));
    Serial.println("Light Intensity = " + static_cast<String>(data.lightIntensity));

    radio.send(TONODEID, (const void*)&data, sizeof(data));
  }

  delay(1000);
}

void scanGPS() {
  gps.scan(350);
}

void measureLightIntensity() {
  lightIntesity = lightMeter.readLightLevel();
  data.lightIntensity = lightIntesity;
}

void measureAirQuality() {
  digitalWrite(AIR_QUALITY_SENSOR_LED_PIN, LOW);
  delayMicroseconds(TIME_OF_MEASUREMENT);
  float measuredVoltage = analogRead(AIR_QUALITY_SENSOR_LED_PIN);
  delayMicroseconds(TIME_OF_EQUALITY);
  digitalWrite(AIR_QUALITY_SENSOR_LED_PIN, HIGH);
  delayMicroseconds(TIME_OF_SLEEP);
  float voltageConversion = measuredVoltage * (3.3 / 1024.0);
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

void measureTemperature() {
  data.temperatureCanSat = bme_cansat.readTemperature();
  data.temperatureExternal = bme.readTemperature();
  data.temperatureMPU = IMU.getTemperature_C();
}

void measurePressure() {
  data.pressureCanSat = bme_cansat.readPressure() / 100.0F;
  data.pressureExternal = bme.readPressure() / 100.0F;
}

void measureHumidity() {
  data.humidityCanSat = bme_cansat.readHumidity();
  data.humidityExternal = bme.readHumidity();
}

void measureAcceleration() {
 data.accelerationX = IMU.getAccelX_mss();
 data.accelerationY = IMU.getAccelY_mss();
 data.accelerationZ = IMU.getAccelZ_mss();
}

void measureRotation() {
  data.rotationX = IMU.getGyroX_rads() * 180 / PI;
  data.rotationY = IMU.getGyroY_rads() * 180 / PI;
  data.rotationZ = IMU.getGyroZ_rads() * 180 / PI;
}
