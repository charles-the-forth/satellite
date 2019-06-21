#include "Arduino.h"

#include "Open_Cansat_GPS.h"
#include "SDCard.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SparkFun_AS7265X.h"
#include <Wire.h>
#include <SPI.h>
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

const int UV_SENSOR_PIN = A2;
const int OXYGEN_SENSOR_PIN = A3;

const int TIME_OF_MEASUREMENT = 280;
const int TIME_OF_EQUALITY = 40;
const int TIME_OF_SLEEP = 9680;

const float VRefer = 5;

OpenCansatGPS gps;
SDCard sdCard;
Adafruit_BME280 bme;
Adafruit_BME280 bme_cansat;
Adafruit_INA219 ina219(0x40);
BH1750 lightMeter;
RFM69 radio(CHIP_SELECT_PIN, INTERUP_PIN, true);
File file;
SCD30 airSensor;
CCS811 myCCS811(CCS811_ADDR);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
//AS7265X spectroscope;

#define Serial SerialUSB

MPU9250 IMU(Wire,0x68);
int status;

typedef struct
{
    uint16_t messageId;
    float temperatureCanSat;
    float temperatureExternal;
    double ambientTemp;
    double objectTemp;
    float pressureCanSat;
    float pressureExternal;
    float humidityCanSat;
    float humidityExternal;
    uint32_t lightIntensity;
    float altitudeCanSat;
    float altitudeExternal;
    uint16_t co2SCD30;
    uint16_t co2CCS811;
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
    float accelerationX;
    float accelerationY;
    float accelerationZ;
    float rotationX;
    float rotationY;
    float rotationZ;
    float magnetometerX;
    float magnetometerY;
    float magnetometerZ;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
    float g;
    float h;
    float i;
    float j;
    float k;
    float l;
    float r;
    float s;
    float t;
    float u;
    float v;
    float w;
} messageOut;

messageOut data;

bool isRadioOk = true;

String csvFilename;

uint16_t year;
uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t minute;
uint8_t second;

#define Serial SerialUSB
bool debugLog = true;

void setup() {
  Serial.begin(57600);

  Serial.println("log!");

  Wire.begin();

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

  CCS811Core::status returnCode = myCCS811.begin();
  Serial.println("CCS811 begin exited with: " + String(returnCode));

  status = IMU.begin();
  if (status < 0 && debugLog) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
  }

  Serial.println("FREQUENCY");
  if(!radio.initialize(FREQUENCY, MYNODEID, NETWORKID))
  {
    Serial.println("init");
    isRadioOk = false;
    if (debugLog) {
      Serial.println("RFM69HW initialization failed!"); 
    }
  }
  else
  {
    Serial.println("error");
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

  mlx.begin();

  /*if(spectroscope.begin() == false)
  {
    Serial.println("Spectroscope does not appear to be connected.");
  }*/
  
  csvFilename = sdCard.getFilename();
  file = SD.open(csvFilename, FILE_WRITE);

  if (file) {
    file.print("message;light;uvIndex;tempCanSat;tempMPU;tempExternal;tempSCD30;ambientTemp;objectTemp;humCanSat;humExternal;humSCD30;pressCanSat;pressExternal;altCanSat;altExternal;accX;accY;accZ;");
    file.println("rotX;rotY;rotZ;magX;magY;magZ;year;month;day;hour;minute;second;numOfSats;latInt;lonInt;latAfterDot;lonAfterDot;voltage_shunt;voltage_bus;current_mA;voltage_load;co2SCD30;co2CCS811;tvoc;o2Con;a;b;c;d;e;f;g;h;i;j;k;l;r;s;t;u;v;w;");
    file.close();
    if (debugLog) {     
      Serial.println("File header written."); 
    }
  } else if (debugLog) {
    Serial.println("Error writing file header.");
  }

  pinMode(D13_led_pin, OUTPUT);

  data.messageId = 0;
}

void loop() {data.messageId++;

  digitalWrite(D13_led_pin, LOW);

  data.lightIntensity = lightMeter.readLightLevel();
  
  data.uvIndex = measureUVSensor();

  data.temperatureCanSat = bme_cansat.readTemperature();
  data.temperatureMPU = IMU.getTemperature_C();
  data.temperatureExternal = bme.readTemperature();

  data.pressureCanSat = bme_cansat.readPressure() / 100.0F;
  data.pressureExternal = bme.readPressure() / 100.0F;

  data.humidityCanSat = bme_cansat.readHumidity();
  data.humidityExternal = bme.readHumidity();

  data.altitudeCanSat = bme_cansat.readAltitude(SEALEVELPRESSURE_HPA);
  data.altitudeExternal = bme.readAltitude(SEALEVELPRESSURE_HPA);

  IMU.readSensor();
  data.accelerationX = IMU.getAccelX_mss();
  data.accelerationY = IMU.getAccelY_mss();
  data.accelerationZ = IMU.getAccelZ_mss();

  data.rotationX = IMU.getGyroX_rads() * 180 / PI;
  data.rotationY = IMU.getGyroY_rads() * 180 / PI;
  data.rotationZ = IMU.getGyroZ_rads() * 180 / PI;

  data.magnetometerX = IMU.getMagX_uT();
  data.magnetometerY = IMU.getMagY_uT();
  data.magnetometerZ = IMU.getMagZ_uT();

  float voltage_shunt = ina219.getShuntVoltage_mV();
  float voltage_bus = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float voltage_load = voltage_bus + (voltage_shunt / 1000);

  data.co2SCD30 = airSensor.getCO2();

  data.temperatureSCD30 = airSensor.getTemperature();

  data.humiditySCD30 = airSensor.getHumidity();

  data.o2Concentration = readConcentration();

  if (myCCS811.dataAvailable())
  {
    myCCS811.readAlgorithmResults();
   
    data.co2CCS811 = myCCS811.getCO2();
    data.tvoc = myCCS811.getTVOC();
    
    myCCS811.setEnvironmentalData(data.humidityExternal, data.temperatureExternal);
  }
  else if (myCCS811.checkForStatusError())
  {
    printSensorError();
  }

  data.ambientTemp = mlx.readAmbientTempC();
  data.objectTemp = mlx.readObjectTempC();

  //spectroscope.takeMeasurements();
  
  /*data.a = spectroscope.getCalibratedA();
  data.b = spectroscope.getCalibratedB();
  data.c = spectroscope.getCalibratedC();
  data.d = spectroscope.getCalibratedD();
  data.e = spectroscope.getCalibratedE();
  data.f = spectroscope.getCalibratedF();
  data.g = spectroscope.getCalibratedG();
  data.h = spectroscope.getCalibratedH();
  data.i = spectroscope.getCalibratedI();
  data.j = spectroscope.getCalibratedJ();
  data.k = spectroscope.getCalibratedK();
  data.l = spectroscope.getCalibratedL();
  data.r = spectroscope.getCalibratedR();
  data.s = spectroscope.getCalibratedS();
  data.t = spectroscope.getCalibratedT();
  data.u = spectroscope.getCalibratedU();
  data.v = spectroscope.getCalibratedV();
  data.w = spectroscope.getCalibratedW();*/
  
  if (gps.scan(350)) {
    year = gps.getYear();
    month = gps.getMonth();
    day = gps.getDay();
    hour = gps.getHour();
    minute = gps.getMinute();
    second = gps.getSecond();
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
    
    Serial.println("UV senzor: " + String(data.uvIndex));

    Serial.println("Temperature CanSat: " + String(data.temperatureCanSat));
    Serial.println("Temperature MPU: " + String(data.temperatureMPU));
    Serial.println("Temperature External: " + String(data.temperatureExternal));
    Serial.println("Temperature SCD30: " + String(data.temperatureSCD30));
    Serial.println("Ambient temperature: " + String(data.ambientTemp));
    Serial.println("Object temperature: " + String(data.objectTemp));
  
    Serial.println("Pressure CanSat: " + String(data.pressureCanSat));
    Serial.println("Pressure External: " + String(data.pressureExternal));

    Serial.println("Humidity CanSat: " + String(data.humidityCanSat));
    Serial.println("Humidity External: " + String(data.humidityExternal));
    Serial.println("Humidity SCD30: " + String(data.humiditySCD30));

    Serial.println("Altitude CanSat: " + String(data.altitudeCanSat));
    Serial.println("Altitude External: " + String(data.altitudeExternal));

    Serial.println("Acceleration X: " + String(data.accelerationX));
    Serial.println("Acceleration Y: " + String(data.accelerationY));
    Serial.println("Acceleration Z: " + String(data.accelerationZ));

    Serial.println("Rotation X: " + String(data.rotationX));
    Serial.println("Rotation Y: " + String(data.rotationY));
    Serial.println("Rotation Z: " + String(data.rotationZ));

    Serial.println("Magnetometer X: " + String(data.magnetometerX));
    Serial.println("Magnetometer Y: " + String(data.magnetometerY));
    Serial.println("Magnetometer Z: " + String(data.magnetometerZ));

    Serial.println("voltage_shunt: " + String(voltage_shunt));
    Serial.println("voltage_bus: " + String(voltage_bus));
    Serial.println("current_mA: " + String(current_mA));
    Serial.println("voltage_load: " + String(voltage_load));
    
    Serial.println("CO2 SCD30: " + String(data.co2SCD30) + " ppm");
    Serial.println("CO2 CCS811: " + String(data.co2CCS811) + " ppm");
    Serial.println("TVOC CCS811: " + String(data.tvoc) + " ppb");
    
    Serial.println("O2: " + String(data.o2Concentration) + " %");

    Serial.println("A: " + String(data.a));
    Serial.println("B: " + String(data.b));
    Serial.println("C: " + String(data.c));
    Serial.println("D: " + String(data.d));
    Serial.println("E: " + String(data.e));
    Serial.println("F: " + String(data.f));
    Serial.println("G: " + String(data.g));
    Serial.println("H: " + String(data.h));
    Serial.println("I: " + String(data.i));
    Serial.println("J: " + String(data.j));
    Serial.println("K: " + String(data.k));
    Serial.println("L: " + String(data.l));
    Serial.println("R: " + String(data.r));
    Serial.println("S: " + String(data.s));
    Serial.println("T: " + String(data.t));
    Serial.println("U: " + String(data.u));
    Serial.println("V: " + String(data.v));
    Serial.println("W: " + String(data.w));
  }

  file = SD.open(csvFilename, FILE_WRITE);
  if (file) {
    file.print(String(data.messageId) + ";" + String(data.lightIntensity) + ";" + String(data.uvIndex) + ";");
    file.print(String(data.temperatureCanSat) + ";" + String(data.temperatureMPU) + ";");
    file.print(String(data.temperatureExternal) + ";" + String(data.temperatureSCD30) + ";" + String(data.ambientTemp) + ";" + String(data.objectTemp) + ";" + String(data.humidityCanSat) + ";"+ String(data.humidityExternal) + ";" + String(data.humiditySCD30) + ";");
    file.print(String(data.pressureCanSat) + ";" + String(data.pressureExternal) + ";");
    file.print(String(data.altitudeCanSat) + ";" + String(data.altitudeExternal) + ";" + String(data.accelerationX)+ ";");
    file.print(String(data.accelerationY) + ";" + String(data.accelerationZ) + ";" + String(data.rotationX) + ";");
    file.print(String(data.rotationY) + ";" + String(data.rotationZ) + ";" + String(data.magnetometerX) + ";");
    file.print(String(data.magnetometerY) + ";" + String(data.magnetometerZ) + ";" + String(year) + ";");
    file.print(String(month) + ";" + String(day) + ";" + String(hour) + ";");
    file.print(String(minute) + ";" + String(second) + ";" + String(data.numberOfSatellites) + ";");
    file.print(String(data.latInt) + ";"  + String(data.lonInt) + ";"  + String(data.latAfterDot) + ";" + String(data.lonAfterDot) + ";");
    file.print(String(voltage_shunt) + ";"  + String(voltage_bus) + ";"  + String(current_mA) + ";" + String(voltage_load) + ";");
    file.print(String(data.co2SCD30) + ";"  + String(data.co2CCS811) + ";"  + String(data.tvoc) + ";"  + String(data.o2Concentration));
    file.println(String(data.a) + ";" + String(data.b) + ";" + String(data.c) + ";" + String(data.d) + ";" + String(data.e) + ";" + String(data.f) + ";" + String(data.g) + ";" + String(data.h) + ";" + String(data.k) + ";" + String(data.l) + ";" + String(data.r) + ";" + String(data.s) + ";" + String(data.t) + ";" + String(data.u) + ";" + String(data.v) + ";" + String(data.w));
    file.close();

    if (debugLog) {     
      Serial.println("Writing was successfull."); 
    }
  } else if (debugLog) {
    Serial.println("Error writing data.");
  }

  digitalWrite(D13_led_pin, HIGH);

  Serial.println("----------------------------------------------------------");
  delay(350);
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

float readO2Vout()
{
    long sum = 0;
    for(int i = 0; i<32; i++)
    {
        sum += analogRead(OXYGEN_SENSOR_PIN);
    }

    sum >>= 5;

    float MeasuredVout = sum * (VRefer / 1023.0);
    return MeasuredVout;
}

float readConcentration()
{
    float MeasuredVout = readO2Vout();

    float Concentration = MeasuredVout * 0.3 / 3.3;
    float Concentration_Percentage=Concentration*100;

    return Concentration_Percentage;
}

void printSensorError()
{
  uint8_t error = myCCS811.getErrorRegister();

  if ( error == 0xFF ) //comm error
  {
    Serial.println("Failed to get ERROR_ID register.");
  }
  else
  {
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
