#include <Arduino.h>
#include "pinLocations.h"
#include <Wire.h>
#include <SparkFunMPL3115A2.h>
#include <SI7021.h>
#include <I2Cdev.h>
#include <HMC5883L.h>
#include <math.h>
#include <NewPing.h>
#include <SPI.h>
#include <SD.h>

// PI constant - overrides PI constant
#define M_PI 3.14159265358979311599796346854418516159057617188
// END Pi

// GLOBAL variables used by the program
uint8_t heading, wHeading, beacon;
// END of GLOBAL variables

// variable used to indicate if the i2C devices are powered
bool i2cPowered = true;
// END of i2C power

// Wire code
bool alive = false;
uint8_t buffer[512];
// END Wire code

// Code for the MPL3115A2 board
MPL3115A2 pressTemp;

char paString[10];
char tempString[10];

double pressure, temperature;
int iPress, iTemp;
// END of altitude/temp sensor

// Code for the Si7021 board
SI7021 humTemp;

double temperature2;
double humidity;
// END code for Si7021

// Code for the HMC5883L board
HMC5883L mag = HMC5883L(HMC5883L_);

//Axis offsets for magnetometer
int xoff = -7;
int yoff = 54;
int zoff = 0;

//Axis scales for magnetometer
float xscale = 1.070;
float yscale = 1.117;
float zscale = 1;

uint8_t sampleSize = 10;
uint8_t sampleRate = 15;

int calibration_cycle = 3; // Value is in minutes
long calibration_time;
int calibration_cycles = 240;

int16_t mx, my, mz;
// END code for HMC5883L

// Code for MaxSonar-EZ0 board
NewPing sonar1(SONAR1TRIGGER, SONAR1ECHO, SONAR1MAXDIST);

uint16_t distance[SONARNUM];
// END code for MaxSonar

// CODE for SD reader/writter
File datafile;
String fileName = "data.txt";
// End code for SD reader/writter

// LOCAL FUNCTIONS
void setupWire()
{
  Wire.begin();
  // Uncomment if unable to connect to I2C network - the line overrides bit rate
  // Wire.setClock(100000L);
  for (uint8_t i =0; i < (sizeof(buffer)/sizeof(buffer[0])); i++)
    buffer[i] = 0;
  alive = true;
}

void shutDownWire()
{
  // Used to lower power consumption
  Wire.end();
  alive = false;
}

void setupMPL3115A2()
{
  if (i2cPowered)
  {
    // WIRE MUST BE STARTED BEFORE THIS WILL RUN
    if (!alive)
      setupWire();
    pressTemp.begin(); // Get sensor online

    pressTemp.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

    pressTemp.setOversampleRate(7); // Set Oversample to the recommended 128
    pressTemp.enableEventFlags(); // Enable all three pressure and temp event flags
  }
}

void gatherpressTempData()
{
  pressure = pressTemp.readPressure();
  iPress = pressure;
  sprintf(paString, "%3d", iPress);

  temperature = pressTemp.readTempF();
  iTemp = temperature;
  sprintf(tempString, "%3d", iTemp);
}

void gatherHumidityTempData()
{
  si7021_env data = humTemp.getHumidityAndTemperature();
  temperature2 = data.celsiusHundredths;
  humidity = data.humidityBasisPoints;
}

void getHeadingData()
{
  mag.getHeading(&mx, &my, &mz);
}

void computeHeading()
{
  int16_t h[sampleSize];
  for (uint8_t i = 0; i < sizeof(h)/sizeof(h[0]); i++)
  {
    getHeadingData();

    mx += xoff;
    my += yoff;
    mz += zoff;

    mx *= xscale;
    my *= yscale;
    mz *= zscale;

    int16_t h_ = atan2(my, mx);
    h_ += M_PI / 2;
    if (h_ < 0)
      h_ += 2 * M_PI;
    if (h_ > 2 * M_PI)
      h_ -= 2 * M_PI;
    h_ *= 180 / M_PI;
    h[i] = h_;
  }
  uint16_t avgH = h[0];
  for (uint8_t i = 1; i < sizeof(h)/sizeof(h[0]); i++)
    avgH = (avgH + h[i]) / 2;

  heading = avgH;
}

void calibrateHMC5883L()
{
  int minX = 0;
  int maxX = 0;

  int minY = 0;
  int maxY = 0;

  int minZ = 0;
  int maxZ = 0;

  int offX = 0;
  int offY = 0;
  int offZ = 0;

  for (int i = 0; i < calibration_cycles; i++)
  {
    getHeadingData();

    if (mx < minX) minX = mx;
    if (mx > maxX) maxX = mx;
    if (my < minY) minY = my;
    if (my > maxY) maxY = my;
    if (mz < minZ) minZ = mz;
    if (mz > maxZ) maxZ = mz;

    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;
    offZ = (maxZ + minZ) / 2;
    delay(sampleRate);
  }

  xoff = offX;
  yoff = offY;
  zoff = offZ;
}

void setupHMC5883L()
{
  if (i2cPowered)
  {
    mag.initialize();

    mag.setMode(HMC5883L_MODE_CONTINUOUS);
    mag.setDataRate(HMC5883L_RATE_75);
    mag.setSampleAveraging(HMC5883L_AVERAGING_8);
    mag.setMeasurementBias(HMC5883L_BIAS_NORMAL);
    mag.setGain(HMC5883L_GAIN_220);
  }
}

void setupSi7021()
{
  humTemp.begin();
}

void getSonarDist(NewPing device, uint8_t pos)
{
  uint16_t d = device.ping_cm(SONAR1MAXDIST);

  distance[pos] = d;
}

void writeSD()
{
  String tmp = "Time," + millis();
  tmp += ",Temp1(MPL3115A2)," + (String)temperature;
  tmp += ",Temp2(Si7021)," + (String)temperature2;
  tmp += ",Humidity(Si7021)," + (String)humidity;
  tmp += ",Pressure(Si7021)," + (String)pressure;
  tmp += ",Heading(HMC5883L)," + (String)heading;
  tmp += ",Beacon(xBee)," + (String)beacon;
  tmp += ",Wanted Heading," + (String)wHeading;
  datafile.println(tmp);
}

void writeSD(String data)
{
  datafile.println(data);
}

void setupSDCARD()
{
  if (!SD.begin(CS, MOSI, MISO, SCK))
    Serial.println("Failed to start SD card.");
}

void go()
{
  wHeading = beacon - 180;
  if (wHeading < 0)
    wHeading += 360;
}
// END LOCAL FUNCTIONS

void setup()
{
  delay(250);

  setupWire();
  setupHMC5883L();
  setupMPL3115A2();
  setupSi7021();

  delay(250);

  writeSD("----START INIT----");

  calibrateHMC5883L();

  delay(1000);

  gatherHumidityTempData();
  gatherpressTempData();
  getHeadingData();

  delay(250);

  computeHeading();

  delay(250);

  writeSD();
  writeSD("----END INIT----");

  delay(1000);
}

void loop()
{

}
