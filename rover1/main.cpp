#include <Arduino.h>
#include "pinlocations.h"
#include <Wire.h>
#include <SparkFunMPL3115A2.h>
#include <SI7021.h>
#include <I2Cdev.h>
#include <HMC5883L.h>
#include <math.h>
#include <NewPing.h>
#include <SPI.h>
#include <SD.h>
#include <XBee.h>

// PI constant - overrides PI constant
#define M_PI 3.14159265358979311599796346854418516159057617188
// END Pi

// GLOBAL variables used by the program
uint8_t heading, wHeading, beacon;
uint8_t tolerance = 5; // number is degrees
// END of GLOBAL variables

// Code for motor controller
#define CW  1
#define CCW 0
#define MOTOR_A 0
#define MOTOR_B 1
const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B
// END OF CONTROLLER

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
#define SONAR A11
long pulse, inches, cm;

uint16_t distance[SONARNUM];
// END code for MaxSonar

// CODE for SD reader/writter
File datafile;
String fileName = "data.txt";
// End code for SD reader/writter

//////////////////////////////////
// xBee Code
XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();
int resetRSSI = -1000;    //The value that RSSI is reset to after each pass through filter
#define samples 110
int temp, smoothData, rawData;
int timeToScan = 2000;
short currentHeading;

//Variable for i2c comms
uint8_t currHeadingI2c[2];

//Structure to contain the readings from the beacon
struct{
  float heading;
  int signalStrength;
} readings[samples];

//Union for converting between byte[4] and float
union{
  float f;
  uint8_t b[4];
} heading_converter;
// END of xBee CODE
//////////////////////////////////

// Timers to run different FUNCTIONS
uint16_t compass = 2500;
uint8_t detection = 120;
unsigned long last_check_compass;
unsigned long last_check_detection;
// END of Timers

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
  // Collects several data points from the compass and computes an average
  // heading
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

void setupXBEE()
{
  Serial1.begin(57600);
  xbee.setSerial(Serial1);
}

void computeNeededHeading()
{
  wHeading = beacon - 180;
  if (wHeading < 0)
    wHeading += 360;
}

void setupArdumoto()
{
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}

void driveArdumoto(byte motor, byte dir, byte spd)
{
  if (motor == MOTOR_A)
  {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  }
  else if (motor == MOTOR_B)
  {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }
}

void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

void forward()
{
  driveArdumoto(MOTOR_A, CCW, 255);
  driveArdumoto(MOTOR_B, CCW, 255);
  delay(250);
}

void left()
{
  driveArdumoto(MOTOR_A, CCW, 255);
  driveArdumoto(MOTOR_B, CW, 255);
  delay(250);
}

void right()
{
  driveArdumoto(MOTOR_A, CW, 255);
  driveArdumoto(MOTOR_B, CCW, 255);
  delay(250);
}

void stop()
{
  driveArdumoto(MOTOR_A, CCW, 0);
  driveArdumoto(MOTOR_B, CCW, 0);
}

void Retrieve(int i){
  xbee.readPacket(10);    //Wait 50 to receive packet
  if (xbee.getResponse().isAvailable())     //Execute only if packet found
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE)
    {
      xbee.getResponse().getRx16Response(rx16);
      //Store the transmitted data and RSSI
      for(int i = 0; i<4; i++)
        heading_converter.b[i] = rx16.getData(i);
      int currentRSSI = -rx16.getRssi();

      //Write to array
      readings[i].heading = heading_converter.f;
      readings[i].signalStrength = currentRSSI;
    }
  }else{
    readings[i].heading = 0;
    readings[i].signalStrength = resetRSSI;
  }
}

int ProcessData(){
  int maxRSSI;
  unsigned long maxIndex = 0;  // THIS LINE DOES NOTHING
  maxRSSI = readings[0].signalStrength;

  //Find max RSSI value
  for (int i=1; i < samples; i++) {
    if (maxRSSI < readings[i].signalStrength) {
      maxRSSI = readings[i].signalStrength;
      maxIndex = i;
    }
  }
  //If there is no valid data
  if(maxRSSI == resetRSSI){
    return -1;
  }

  float headingx = 0;
  float headingy = 0;
  for(int i = 0; i < samples; i++)
  {
    if (readings[i].signalStrength == -1000 && readings[i].heading == 0)
    {
       Serial.println("this heading not included");
    }
    else
    {
      Serial.print(readings[i].heading);
      Serial.print("\t");
      Serial.println(readings[i].signalStrength);
      // Set magnitude of vector by signal strength
      headingx += readings[i].signalStrength * cos(readings[i].heading * PI / 180);
      headingy += readings[i].signalStrength * sin(readings[i].heading * PI / 180);
    }
  }

  float heading = atan2(headingy, headingx);
  if (heading < 0) heading += 2 * PI;
  heading = heading * 180 / PI;

  return (int) heading;
}

void getSamples()
{
  for(int i = 0;i<samples;i++){
    Retrieve(i);
    float propComplete = ((float)i)/(float)samples; // this line does NOTHING
    delay(timeToScan/samples);
  }
}
// END LOCAL FUNCTIONS

void setup()
{
  delay(250);

  setupWire();
  setupHMC5883L();
  setupMPL3115A2();
  setupSi7021();
  //setupSDCARD();

  pinMode(SONAR, INPUT);

  delay(250);

  //writeSD("----START INIT----");

  calibrateHMC5883L();

  delay(1000);

  gatherHumidityTempData();
  gatherpressTempData();
  getHeadingData();

  delay(250);

  computeHeading();

  delay(250);

  //writeSD();
  //writeSD("----END INIT----");

  delay(1000);

  last_check_compass = millis();
  last_check_detection = last_check_compass;
}

void loop()
{
  unsigned long time_ = millis();

  if (time_ - last_check_compass > compass)
  {
    stop();
    getSamples();
    last_check_compass = millis();
  }
  if (time_ - last_check_detection > detection)
  {
    computeHeading();
    computeNeededHeading();
    getSonarDist(sonar1, 0);
    pulse = pulseIn(SONAR, HIGH);
    inches = pulse / 147;
    cm = inches * 2.54;
    stop();
    CHECK:if (inches < 10)
    {
      right();
      goto CHECK;
    }
    else
      if (abs(beacon - wHeading) < tolerance)
        forward();
      else
        if (wHeading - 180 < 0)
          right();
        else
          left();
    last_check_detection = millis();
  }
}
