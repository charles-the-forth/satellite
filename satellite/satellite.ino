#include "Arduino.h"

#include "Open_Cansat_GPS.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SparkFun_AS7265X.h"
#include <Wire.h>
#include <SPI.h>
#include <SDCard.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <MPU9250.h>
#include <RFM69.h>
#include <Adafruit_INA219.h>
#include <SparkFunCCS811.h>
#include <Adafruit_MLX90614.h>

#define CCS811_ADDR 0x5B
#define BME280_ADRESS 0x76
#define BME280_ADDRESS_OPEN_CANSAT 0x77
#define SEALEVELPRESSURE_HPA 1013.25

// RFM69
#define NETWORKID       0
#define MYNODEID        1
#define TONODEID        2
#define FREQUENCY       RF69_433MHZ
#define FREQUENCYSPECIFIC 443000000
#define CHIP_SELECT_PIN   43
#define INTERUP_PIN       9
#define sd_cs_pin 35

#define D13_led_pin 42
#define Serial SerialUSB

const int UV_SENSOR_PIN = A1;
const int OXYGEN_SENSOR_PIN = A3;

const int TIME_OF_MEASUREMENT = 280;
const int TIME_OF_EQUALITY = 40;
const int TIME_OF_SLEEP = 9680;

const int SERIAL_PORT = 57600;

const float VRefer = 5;

OpenCansatGPS gps;
Adafruit_BME280 bme;
Adafruit_BME280 bme_cansat;
Adafruit_INA219 ina219(0x40);
BH1750 lightMeter;
RFM69 radio(CHIP_SELECT_PIN, INTERUP_PIN, true);
SDCard sdCard;
File file;
SCD30 airSensor;
CCS811 myCCS811(CCS811_ADDR);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
AS7265X spectroscope;
MPU9250 IMU(Wire,0x68);

uint32_t messageId;
uint8_t messageType;
uint8_t messageCounter;

typedef struct
{
  uint8_t messageType;
  uint16_t messageId;
  float temperatureCanSat;
  float temperatureExternal;
  float ambientTemp;
  float objectTemp;
  float pressureCanSat;
  float pressureExternal;
  float humidityCanSat;
  float humidityExternal;
  uint32_t lightIntensity;
  float altitudeCanSat;
  float altitudeExternal;
  uint16_t co2SCD30;
  uint16_t co2CCS811;
} message1;
  
typedef struct
{
  uint8_t messageType;
  uint16_t messageId;
  uint16_t tvoc;
  float o2Concentration;
  uint16_t latInt;
  uint16_t lonInt;
  uint32_t latAfterDot;
  uint32_t lonAfterDot;
  uint8_t numberOfSatellites;
  float uvIndex;
  float temperatureMPU;
  float temperatureSCD30;
  float humiditySCD30;
} message2;
  
typedef struct
{
  uint8_t messageType;
  uint16_t messageId;   
  float accelerationX;
  float accelerationY;
  float accelerationZ;
  float rotationX;
  float rotationY;
  float rotationZ;
  float magnetometerX;
  float magnetometerY;
  float magnetometerZ;
} message3;

typedef struct {
  uint8_t messageType;
  uint16_t messageId;
  float a;
  float b;
  float c;
  float d;
  float e;
  float f;
  float g;
  float h;
  float r;
  float i;
  float s;
  float j;
  float t;
} message4;

message1 data1;
message2 data2;
message3 data3;
message4 data4;

float k;
float u;
float v;
float w;
float l;

bool isRadioOk = true;
bool isSDCardInitialised = true;

String filename;

uint16_t year;
uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t minute;
uint8_t second;

bool debugLog = true;

float voltage_shunt;
float voltage_bus;
float current_mA;
float voltage_load;

void setup() {
  Serial.begin(SERIAL_PORT);
  
  if (debugLog) {
    Serial.println("--------------------   Setup start   --------------------");
  }

  Wire.begin();

  initGPS();
  initExternalBME();
  initInternalBME();
  initAirSensor();
  initCCS811();
  initIMU();
  initRadio();
  initAdafruitMLX();
  initSpectroscope();
  initSDCard();

  lightMeter.begin();

  ina219.begin();

  if (isSDCardInitialised) {
    writeFileHeader();
  } else if (debugLog) {
    Serial.println("File header could not be written because SD card is not initialised!");
  }

  pinMode(D13_led_pin, OUTPUT);

  messageId = 0;
  messageType = 1;
  messageCounter = 0;
  
  if (debugLog) {
    Serial.println("--------------------    Setup end    --------------------");
  }
}

void loop() {
  messageId++;

  if (isSDCardInitialised) {
    writeDataToSDCard();
  }

  digitalWrite(D13_led_pin, LOW);

  float voltage_shunt = ina219.getShuntVoltage_mV();
  float voltage_bus = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float voltage_load = voltage_bus + (voltage_shunt / 1000);

  if (messageType == 1) {
    IMU.readSensor();
    data1.messageType = 1;
    data1.messageId = messageId;
    data1.temperatureCanSat = bme_cansat.readTemperature();
    data1.temperatureExternal = bme.readTemperature();
    data1.ambientTemp = mlx.readAmbientTempC();
    data1.objectTemp = mlx.readObjectTempC();
    data1.pressureCanSat = bme_cansat.readPressure() / 100.0F;
    data1.pressureExternal = bme.readPressure() / 100.0F;
    data1.humidityCanSat = bme_cansat.readHumidity();
    data1.humidityExternal = bme.readHumidity();
    data1.lightIntensity = lightMeter.readLightLevel();
    data1.altitudeCanSat = bme_cansat.readAltitude(SEALEVELPRESSURE_HPA);
    data1.altitudeExternal = bme.readAltitude(SEALEVELPRESSURE_HPA);

    if (airSensor.dataAvailable()) {
      data1.co2SCD30 = airSensor.getCO2();
    }
    
    if (myCCS811.dataAvailable()) {
      myCCS811.readAlgorithmResults();
      data1.co2CCS811 = myCCS811.getCO2();
      data2.tvoc = myCCS811.getTVOC();
    
      myCCS811.setEnvironmentalData(data1.humidityExternal, data1.temperatureExternal);
    } else if (myCCS811.checkForStatusError()) {
      printSensorError();
    }
  
    if(isRadioOk) {
      radio.send(TONODEID, (const void*)&data1, sizeof(data1));
      
      if (debugLog) {
        Serial.println("message1 sent.");
      }
    }

    messageCounter++;
  } else if (messageType == 2) {
    data2.messageType = 2;
    data2.messageId = messageId;
    data2.o2Concentration = readConcentration();
    data2.uvIndex = measureUVSensor();
    data2.temperatureMPU = IMU.getTemperature_C();

    if (airSensor.dataAvailable()) {
      data2.temperatureSCD30 = airSensor.getTemperature();
      data2.humiditySCD30 = airSensor.getHumidity();
    }

    if (gps.scan(350)) {
      year = gps.getYear();
      month = gps.getMonth();
      day = gps.getDay();
      hour = gps.getHour();
      minute = gps.getMinute();
      second = gps.getSecond();
      data2.numberOfSatellites = gps.getNumberOfSatellites();
      data2.latInt = gps.getLatInt();
      data2.lonInt = gps.getLonInt();
      data2.latAfterDot = gps.getLatAfterDot();
      data2.lonAfterDot = gps.getLonAfterDot();
    }
  
    if(isRadioOk) {
      radio.send(TONODEID, (const void*)&data2, sizeof(data2));
      
      if (debugLog) {
        Serial.println("message2 sent.");
      }
    }

    messageCounter++;
  } else if (messageType == 3) {
    IMU.readSensor();
    data3.messageType = 3;
    data3.messageId = messageId;   
    data3.accelerationX = IMU.getAccelX_mss();
    data3.accelerationY = IMU.getAccelY_mss();
    data3.accelerationZ = IMU.getAccelZ_mss();
    
    data3.rotationX = IMU.getGyroX_rads() * 180 / PI;
    data3.rotationY = IMU.getGyroY_rads() * 180 / PI;
    data3.rotationZ = IMU.getGyroZ_rads() * 180 / PI;
    
    data3.magnetometerX = IMU.getMagX_uT();
    data3.magnetometerY = IMU.getMagY_uT();
    data3.magnetometerZ = IMU.getMagZ_uT();
  
    if(isRadioOk) {
      radio.send(TONODEID, (const void*)&data3, sizeof(data3));
      
      if (debugLog) {
        Serial.println("message3 sent.");
      }
    }

    messageCounter++;
  } else {
    data4.messageType = 4;
    data4.messageId = messageId;
    spectroscope.takeMeasurements();

    data4.a = spectroscope.getCalibratedA();
    data4.b = spectroscope.getCalibratedB();
    data4.c = spectroscope.getCalibratedC();
    data4.d = spectroscope.getCalibratedD();
    data4.e = spectroscope.getCalibratedE();
    data4.f = spectroscope.getCalibratedF();
    data4.g = spectroscope.getCalibratedG();
    data4.h = spectroscope.getCalibratedH();
    data4.i = spectroscope.getCalibratedI();
    data4.j = spectroscope.getCalibratedJ();
    k = spectroscope.getCalibratedK();
    l = spectroscope.getCalibratedL();
    data4.r = spectroscope.getCalibratedR();
    data4.s = spectroscope.getCalibratedS();
    data4.t = spectroscope.getCalibratedT();
    u = spectroscope.getCalibratedU();
    v = spectroscope.getCalibratedV();
    w = spectroscope.getCalibratedW();
  
    if(isRadioOk) {
      radio.send(TONODEID, (const void*)&data4, sizeof(data4));

      if (debugLog) {
        Serial.println("message4 sent.");
      }
    }

    messageCounter++;
  }

  if (debugLog) {
    printlnMeasuredData();
  }

  digitalWrite(D13_led_pin, HIGH);

  if (messageCounter == 2) {   
    messageType++; 
    messageCounter = 0;
  }

  if (messageType == 5) {
    messageType = 1;
  }
  
  Serial.println("---------------------------------------------------------");
  delay(150);
}

float measureUVSensor() {
  float uvSensorValue = 0;
  int uvAnalog = analogRead(UV_SENSOR_PIN);
  
  if (debugLog) {
    Serial.println("UV sensor analog value: " + String(uvAnalog));
  }  
  
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

float readO2Vout() {
    long sum = 0;
    for(int i = 0; i < 32; i++) {
        sum += analogRead(OXYGEN_SENSOR_PIN);
    }

    sum >>= 5;

    float MeasuredVout = sum * (VRefer / 1023.0);
    return MeasuredVout;
}

float readConcentration() {
    float MeasuredVout = readO2Vout();

    float Concentration = MeasuredVout * 0.3 / 3.3;
    float Concentration_Percentage = Concentration * 100;

    return Concentration_Percentage;
}

void printSensorError() {
  uint8_t error = myCCS811.getErrorRegister();

  if ( error == 0xFF ) {
    Serial.println("Failed to get ERROR_ID register.");
  } else {
    Serial.print("Error: ");
    if (error & 1 << 5) Serial.print("HeaterSupply");
    if (error & 1 << 4) Serial.print("HeaterFault");
    if (error & 1 << 3) Serial.print("MaxResistance");
    if (error & 1 << 2) Serial.print("MeasModeInvalid");
    if (error & 1 << 1) Serial.print("ReadRegInvalid");
    if (error & 1 << 0) Serial.print("MsgInvalid");
    Serial.println();
  }
}

void initGPS() {
  gps.begin();

  if (debugLog) {
    gps.debugPrintOn(SERIAL_PORT);
  }
}

void initExternalBME() {
  bool initialised = bme.begin(BME280_ADRESS);
  
  if (initialised && debugLog) {
    Serial.println("External BME280 initialisation successful.");
  } else if (debugLog) {
    Serial.println("External BME280 initialisation failed!");
  }
}

void initInternalBME() {
  bool initialised = bme_cansat.begin(BME280_ADDRESS_OPEN_CANSAT);

  if (initialised && debugLog) {
    Serial.println("Internal BME280 initialisation successful.");
  } else if (debugLog) {
    Serial.println("Internal BME280 initialisation failed!");
  }
}

void initAirSensor() {
  bool initialised = airSensor.begin();

  if (initialised && debugLog) {
    Serial.println("SCD30 sensor initialisation successful.");
  } else if (debugLog) {
    Serial.println("SCD30 sensor initialisation failed!");
  }
}

void initCCS811() {
  bool initialised = myCCS811.begin();

  if (initialised && debugLog) {
    Serial.println("CCS811 sensor initialisation successful.");
  } else if (debugLog) {
    Serial.println("CCS811 sensor initialisation failed!");
  }
}

void initIMU() {
  int status = IMU.begin();

  if (status < 0 && debugLog) {
    Serial.println("IMU initialisation unsuccessful.");
  } else if (debugLog) {
    Serial.println("IMU initialisation successful!");
  }
}

void initRadio() {
  bool initialised = radio.initialize(FREQUENCY, MYNODEID, NETWORKID);

  if (initialised) {
    radio.setFrequency(FREQUENCYSPECIFIC);
    radio.setHighPower(true);

    if (debugLog) {
      Serial.println("RFM69HW initialisation successful."); 
    }
  } else {
    isRadioOk = false;

    if (debugLog) {
      Serial.println("RFM69HW initialisation failed!"); 
    }
  }
}

void initSDCard() {
  isSDCardInitialised = SD.begin(sd_cs_pin);

  if (isSDCardInitialised && debugLog) {
    Serial.println("SD card initialisation successful.");
  } else if (debugLog) {   
    Serial.println("SD card initialisation failed!"); 
  }
}

void initAdafruitMLX() {
  bool initialised = mlx.begin();

  if (initialised && debugLog) {
    Serial.println("Adafruit MLX initialisation successful.");
  } else if (debugLog) {
    Serial.println("Adafruit MLX initialisation failed!");
  }
}

void initSpectroscope() {
  bool initialised = spectroscope.begin();

  if (initialised && debugLog) {
    Serial.println("Spectroscope initialisation successful.");
  } else if (debugLog) {
    Serial.println("Spectroscope initialisation failed!");
  }
}

void writeFileHeader() {
  filename = SDCard::getFilename();
  file = SD.open(filename, FILE_WRITE);
 
  if (file) {
    file.print("message;light;uvIndex;tempCanSat;tempMPU;tempExternal;tempSCD30;ambientTemp;");
    file.print("objectTemp;humCanSat;humExternal;humSCD30;pressCanSat;pressExternal;");
    file.print("altCanSat;altExternal;accX;accY;accZ;");
    file.print("rotX;rotY;rotZ;magX;magY;magZ;year;month;day;");
    file.print("hour;minute;second;numOfSats;latInt;lonInt;latAfterDot;");
    file.print("lonAfterDot;voltage_shunt;voltage_bus;current_mA;");
    file.println("voltage_load;co2SCD30;co2CCS811;tvoc;o2Con;a;b;c;d;e;f;g;h;i;j;k;l;r;s;t;u;v;w;");
    file.close();
    
    if (debugLog) {     
      Serial.println("File header written."); 
    }
  } else if (debugLog) {
    Serial.println("Error opening file!");
  }
}

void writeDataToSDCard() {
  file = SD.open(filename, FILE_WRITE);

  if (debugLog) {
    Serial.println("File: " + String(file));
    Serial.println("Filename: " + filename);  
  }

  if (file) {
    String message = String(messageId) + ";" + String(data1.lightIntensity) + ";" + String(data2.uvIndex) + ";" + String(data1.temperatureCanSat) + ";" + String(data2.temperatureMPU) + ";"
      + String(data1.temperatureExternal) + ";" + String(data2.temperatureSCD30) + ";" + String(data1.ambientTemp) + ";" + String(data1.objectTemp) + ";" + String(data1.humidityCanSat) + ";"+ String(data1.humidityExternal) + ";" + String(data2.humiditySCD30) + ";"
      + String(data1.pressureCanSat) + ";" + String(data1.pressureExternal) + ";" + String(data1.altitudeCanSat) + ";" + String(data1.altitudeExternal) + ";" + String(data3.accelerationX)+ ";"
      + String(data3.accelerationY) + ";" + String(data3.accelerationZ) + ";" + String(data3.rotationX) + ";" + String(data3.rotationY) + ";" + String(data3.rotationZ) + ";" + String(data3.magnetometerX) + ";";
     String message1 = String(data3.magnetometerY) + ";" + String(data3.magnetometerZ) + ";" + String(year) + ";" + String(month) + ";" + String(day) + ";" + String(hour) + ";" + String(minute) + ";" + String(second) + ";" + String(data2.numberOfSatellites) + ";"
      + String(data2.latInt) + ";"  + String(data2.lonInt) + ";"  + String(data2.latAfterDot) + ";" + String(data2.lonAfterDot) + ";" + String(voltage_shunt) + ";"  + String(voltage_bus) + ";"  + String(current_mA) + ";" + String(voltage_load) + ";"
      + String(data1.co2SCD30) + ";"  + String(data1.co2CCS811) + ";"  + String(data2.tvoc) + ";"  + String(data2.o2Concentration) + ";"
      + String(data4.a) + ";" + String(data4.b) + ";" + String(data4.c) + ";";
     String message2 = String(data4.d) + ";" + String(data4.e) + ";" + String(data4.f) + ";"
      + String(data4.g) + ";" + String(data4.h) + ";" + String(data4.i) + ";"
      + String(data4.j) + ";" + String(k) + ";" + String(l) + ";" + String(data4.r) + ";" + String(data4.s) + ";" + String(data4.t) + ";" + String(u) + ";" + String(v) + ";" + String(w);
    file.print(message);
    file.print(message1);
    file.println(message2);
    file.flush();
    file.close();

    if (debugLog) {     
      Serial.println("Writing was successfull."); 
    }
  } else if (debugLog) {
    Serial.println("Error writing data.");
    delay(150);
  }  
}

void printlnMeasuredData() {
  Serial.println("Message id: " + String(messageId));
  
  Serial.println("Light intensity: " + String(data1.lightIntensity));
  
  Serial.println("UV sensor: " + String(data2.uvIndex));

  Serial.println("Temperature CanSat: " + String(data1.temperatureCanSat));
  Serial.println("Temperature MPU: " + String(data2.temperatureMPU));
  Serial.println("Temperature External: " + String(data1.temperatureExternal));
  Serial.println("Temperature SCD30: " + String(data2.temperatureSCD30));
  Serial.println("Ambient temperature: " + String(data1.ambientTemp));
  Serial.println("Object temperature: " + String(data1.objectTemp));

  Serial.println("Pressure CanSat: " + String(data1.pressureCanSat));
  Serial.println("Pressure External: " + String(data1.pressureExternal));

  Serial.println("Humidity CanSat: " + String(data1.humidityCanSat));
  Serial.println("Humidity External: " + String(data1.humidityExternal));
  Serial.println("Humidity SCD30: " + String(data2.humiditySCD30));

  Serial.println("Altitude CanSat: " + String(data1.altitudeCanSat));
  Serial.println("Altitude External: " + String(data1.altitudeExternal));

  Serial.println("O2: " + String(data2.o2Concentration) + " %");
  Serial.println("Acceleration X: " + String(data3.accelerationX));
  Serial.println("Acceleration Y: " + String(data3.accelerationY));
  Serial.println("Acceleration Z: " + String(data3.accelerationZ));

  Serial.println("Rotation X: " + String(data3.rotationX));
  Serial.println("Rotation Y: " + String(data3.rotationY));
  Serial.println("Rotation Z: " + String(data3.rotationZ));

  Serial.println("Magnetometer X: " + String(data3.magnetometerX));
  Serial.println("Magnetometer Y: " + String(data3.magnetometerY));
  Serial.println("Magnetometer Z: " + String(data3.magnetometerZ));

  Serial.println("voltage_shunt: " + String(voltage_shunt));
  Serial.println("voltage_bus: " + String(voltage_bus));
  Serial.println("current_mA: " + String(current_mA));
  Serial.println("voltage_load: " + String(voltage_load));

  Serial.println("CO2 SCD30: " + String(data1.co2SCD30) + " ppm");
  Serial.println("CO2 CCS811: " + String(data1.co2CCS811) + " ppm");
  Serial.println("TVOC CCS811: " + String(data2.tvoc) + " ppb");

  Serial.println("A: " + String(data4.a));
  Serial.println("B: " + String(data4.b));
  Serial.println("C: " + String(data4.c));
  Serial.println("D: " + String(data4.d));
  Serial.println("E: " + String(data4.e));
  Serial.println("F: " + String(data4.f));
  Serial.println("G: " + String(data4.g));
  Serial.println("H: " + String(data4.h));
  Serial.println("I: " + String(data4.i));
  Serial.println("J: " + String(data4.j));
  Serial.println("K: " + String(k));
  Serial.println("L: " + String(l));
  Serial.println("R: " + String(data4.r));
  Serial.println("S: " + String(data4.s));
  Serial.println("T: " + String(data4.t));
  Serial.println("U: " + String(u));
  Serial.println("V: " + String(v));
  Serial.println("W: " + String(w));
}
