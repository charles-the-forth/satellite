#include "Arduino.h"

#include "Open_Cansat_GPS.h"
#include "SDCard.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <MPU9250.h>
#include <RFM69.h>
#include <Adafruit_INA219.h>

#define BME280_ADRESS 0x76
#define BME280_ADDRESS_OPEN_CANSAT 0x77
#define SEALEVELPRESSURE_HPA 1013.25

// RFM69
#define NETWORKID       0
#define MYNODEID        1
#define TONODEID        2
#define FREQUENCY       RF69_433MHZ
#define FREQUENCYSPECIFIC 433102000
#define CHIP_SELECT_PIN   43
#define INTERUP_PIN       9
#define sd_cs_pin 35

const int AIR_QUALITY_SENSOR_PIN = A0;
const int CAPACITIVE_SOIL_MOISTURE_SENSOR_PIN = A1;
const int UV_SENSOR_PIN = A2;
const int AIR_QUALITY_SENSOR_LED_PIN = 3;

const int TIME_OF_MEASUREMENT = 280;
const int TIME_OF_EQUALITY = 40;
const int TIME_OF_SLEEP = 9680;

OpenCansatGPS gps;
SDCard sdCard;
Adafruit_BME280 bme;
Adafruit_BME280 bme_cansat;
Adafruit_INA219 ina219(0x40);
BH1750 lightMeter;
RFM69 radio(CHIP_SELECT_PIN, INTERUP_PIN, true);
File file;
SCD30 airSensor;

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
    uint16_t latInt;
    uint16_t lonInt;
    uint32_t latAfterDot;
    uint32_t lonAfterDot;
} messageOut;

messageOut data;

bool isRadioOk = true;

String csvFilename;

#define Serial SerialUSB
bool debugLog = true;

void setup() {
  Serial.begin(57600);

  gps.begin();
  //gps.debugPrintOn(57600);
  
  if (!bme.begin(BME280_ADRESS) && debugLog) {
    Serial.println("External BME280 not found!");
  }
  
  if (!bme_cansat.begin(BME280_ADDRESS_OPEN_CANSAT) && debugLog) {
    Serial.println("Internal BME280 not found!");
  }
  
  lightMeter.begin();

  ina219.begin();

  airSensor.begin();

  status = IMU.begin();
  if (status < 0 && debugLog) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
  }

  if(!radio.initialize(FREQUENCY, MYNODEID, NETWORKID))
  {
    isRadioOk = false;
    if (debugLog) {
      Serial.println("RFM69HW initialization failed!"); 
    }
  }
  else
  {
    radio.setFrequency(FREQUENCYSPECIFIC);
    radio.setHighPower(true);
    if (debugLog) {
      Serial.println("RFM69HW successful!"); 
    }
  }

  if (!SD.begin(sd_cs_pin) && debugLog) {
    Serial.println("initialization failed!");
  } else if (debugLog) {   
    Serial.println("initialization done."); 
  }

  csvFilename = sdCard.getFilename();
  file = SD.open(csvFilename, FILE_WRITE);

  if (file) {
    file.print("message;light;capacity;uvIndex;tempCanSat;tempMPU;tempExternal;humCanSat;humExternal;airQuality;pressCanSat;pressExternal;altCanSat;altExternal;accX;accY;accZ;");
    file.println("rotX;rotY;rotZ;magX;magY;magZ;year;month;day;hour;minute;second;numOfSats;latInt;lonInt;latAfterDot;lonAfterDot;voltage_shunt;voltage_bus;current_mA;voltage_load");
    file.close();
    if (debugLog) {     
      Serial.println("File header written."); 
    }
  } else if (debugLog) {
    Serial.println("Error writing file header.");
  }

  data.messageId = 0;
}

void loop() {
  data.messageId++;

  data.lightIntensity = lightMeter.readLightLevel();
  
  uint8_t capacitiveSoilMoistureSensorValue = analogRead(CAPACITIVE_SOIL_MOISTURE_SENSOR_PIN);

  float uvIndex = measureUVSensor();

  data.temperatureCanSat = bme_cansat.readTemperature();
  float temperatureMPU = IMU.getTemperature_C();
  float temperatureExternal = bme.readTemperature();

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

  float voltage_shunt = ina219.getShuntVoltage_mV();
  float voltage_bus = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float voltage_load = voltage_bus + (voltage_shunt / 1000);

  float airQuality = measureAirQuality(voltage_load);

  int co2 = airSensor.getCO2();

  float temperatureCO2 = airSensor.getTemperature();

  float humidityCO2 = airSensor.getHumidity();

  if (gps.scan(350)) {
    data.year = gps.getYear();
    data.month = gps.getMonth();
    data.day = gps.getDay();
    data.hour = gps.getHour();
    data.minute = gps.getMinute();
    data.second = gps.getSecond();
    data.numberOfSatellites = gps.getNumberOfSatellites();
    data.latInt = gps.getLatInt();
    data.lonInt = gps.getLonInt();
    data.latAfterDot = gps.getLatAfterDot();
    data.lonAfterDot = gps.getLonAfterDot();
  }
  
  if(isRadioOk)
  {
    radio.send(TONODEID, (const void*)&data, sizeof(data));
  }

  if (debugLog) {
    Serial.println("Message id: " + String(data.messageId));
  
    Serial.println("Light intensity: " + String(data.lightIntensity));
  
    Serial.println("Capacitive soil moisture: " + String(capacitiveSoilMoistureSensorValue));
    
    Serial.println("UV senzor: " + String(uvIndex));

    Serial.println("Temperature CanSat: " + String(data.temperatureCanSat));
    Serial.println("Temperature MPU: " + String(temperatureMPU));
    Serial.println("Temperature External: " + String(temperatureExternal));
  
    Serial.println("Pressure CanSat: " + String(data.pressureCanSat));
    Serial.println("Pressure External: " + String(pressureExternal));

    Serial.println("Humidity CanSat: " + String(data.humidityCanSat));
    Serial.println("Humidity External: " + String(humidityExternal));

    Serial.println("Altitude CanSat: " + String(data.altitudeCanSat));
    Serial.println("Altitude External: " + String(altitudeExternal));

    Serial.println("Acceleration X: " + String(accelerationX));
    Serial.println("Acceleration Y: " + String(accelerationY));
    Serial.println("Acceleration Z: " + String(accelerationZ));

    Serial.println("Rotation X: " + String(rotationX));
    Serial.println("Rotation Y: " + String(rotationY));
    Serial.println("Rotation Z: " + String(rotationZ));

    Serial.println("Magnetometer X: " + String(magnetometerX));
    Serial.println("Magnetometer Y: " + String(magnetometerY));
    Serial.println("Magnetometer Z: " + String(magnetometerZ));

    Serial.println("voltage_shunt: " + String(voltage_shunt));
    Serial.println("voltage_bus: " + String(voltage_bus));
    Serial.println("current_mA: " + String(current_mA));
    Serial.println("voltage_load: " + String(voltage_load));
  
    Serial.println("Air quality: " + String(airQuality));
    
    Serial.println("CO2: " + String(co2) + " ppm");
    Serial.println("Temperature CO2: " + String(temperatureCO2, 1) + " C");
    Serial.println("Humidity CO2: " + String(humidityCO2, 1) + " %");    
  }

  file = SD.open(csvFilename, FILE_WRITE);
  if (file) {
    file.print(String(data.messageId) + ";" + String(data.lightIntensity) + ";" + String(capacitiveSoilMoistureSensorValue) + ";");
    file.print(String(uvIndex) + ";" + String(data.temperatureCanSat) + ";" + String(temperatureMPU) + ";");
    file.print(String(temperatureExternal) + ";" + String(data.humidityCanSat) + ";"+ String(humidityExternal) + ";");
    file.print(String(airQuality) + ";" + String(data.pressureCanSat) + ";" + String(pressureExternal) + ";");
    file.print(String(data.altitudeCanSat) + ";" + String(altitudeExternal) + ";" + String(accelerationX)+ ";");
    file.print(String(accelerationY) + ";" + String(accelerationZ) + ";" + String(rotationX) + ";");
    file.print(String(rotationY) + ";" + String(rotationZ) + ";" + String(magnetometerX) + ";");
    file.print(String(magnetometerY) + ";" + String(magnetometerZ) + ";" + String(data.year) + ";");
    file.print(String(data.month) + ";" + String(data.day) + ";" + String(data.hour) + ";");
    file.print(String(data.minute) + ";" + String(data.second) + ";" + String(data.numberOfSatellites) + ";");
    file.print(String(data.latInt) + ";"  + String(data.lonInt) + ";"  + String(data.latAfterDot) + ";" + String(data.lonAfterDot) + ";");
    file.println(String(voltage_shunt) + ";"  + String(voltage_bus) + ";"  + String(current_mA) + ";" + String(voltage_load));
    file.close();

    if (debugLog) {     
      Serial.println("Writing was successfull."); 
    }
  } else if (debugLog) {
    Serial.println("Error writing data.");
  }

  Serial.println("----------------------------------------------------------");
  delay(350);
}

float measureAirQuality(float voltage) {
  digitalWrite(AIR_QUALITY_SENSOR_LED_PIN, LOW);
  delayMicroseconds(TIME_OF_MEASUREMENT);
  float measuredVoltage = analogRead(AIR_QUALITY_SENSOR_LED_PIN);
  delayMicroseconds(TIME_OF_EQUALITY);
  digitalWrite(AIR_QUALITY_SENSOR_LED_PIN, HIGH);
  delayMicroseconds(TIME_OF_SLEEP);
  float convertedVoltage = measuredVoltage * (voltage / 1024.0);
  return (0.17 * convertedVoltage - 0.1) * 1000;
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
