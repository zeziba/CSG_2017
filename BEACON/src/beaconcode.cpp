/*
 * Created By: Charles Engen
 * Modified Friday March 20, 2017
 *
 * Built for Colorado Space Grant Consortium some code extracts were used from
 * https://github.com/Colorado-Space-Grant-Consortium/Robotics_Challenge/blob/2016_master/2017_Robotics_Challenge/xb_Transmitter_System_Code/xb_TX_Combo/xb_TX_Combo.ino
 * to match the system used
 *  Finalized by Jack Maydan based off Adam St. Amard's earlier versions.
 *  Edited by Robert Belter 10/30/2015
 *  Updated by Chase Pellazar 02/09/2017
 *
 * This program will control the Beacon. The Beacon will transmit the Beacons current heading and it will also disply the current heading in the lcd screen attached.
 * This program works based on the Spark Fun Arduino Uno v5.0 with an XBEE transmitter hooked to an extended antennae which is mounted on an xBee shield.
 * The board also is hooked to a 3 axis magnetometer.
 *
 *
    Copyright 2017 Charles Engen

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to
    whom the Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
    OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
    USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * This code is open source and free to use, its derivatives should follow the same guidelines.
 */
#include <Wire.h>
#include <XBee.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Arduino.h>

// Adress location for magnetometer
// The magnometer can only communicate at 160hz max, this translates into a 0.00625 second delay between each signal
// This translates into a max of 160 readings per second
#define Mag 0x1E

// PI constant
#define M_PI 3.14159265358979311599796346854418516159057617188

double declination = -8.51;  // Declination around Pueblo Colorado

// Starts the lcd display pinout
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//Axis offsets for magnetometer
int xoff = -7;
int yoff = 54;
int zoff = 0;

//Axis scales for magnetometer
float xscale = 1.070;
float yscale = 1.117;
float zscale = 1;

// The below Code will be used to calculate the amount of time that is required for each read of the magometer and when it sends the data
float RPM = 1.0;

SoftwareSerial xBEE(2, 3); // TX, RX lines
XBee xBee = XBee();
int resetRSSI = -1000; // Reset RSSI number

double lastRun; // Variable to indicate last time run
double lastSend; // Varialbe to indicate last sent bytes

union{
  float f;
  uint8_t b[4];
}heading_converter; // Current heading

// Below Code allows the beacon to output the systems RPM allowing the user to calibrate the speed of rotation.

boolean cont = true;  // Use to turn off and on the continuous read mode
typedef enum
{
    DATARATE_75HZ       = 0b110,
    DATARATE_30HZ       = 0b101,
    DATARATE_15HZ       = 0b100,
    DATARATE_7_5HZ      = 0b011,
    DATARATE_3HZ        = 0b010,
    DATARATE_1_5HZ      = 0b001,
    DATARATE_0_75_HZ    = 0b000
} dataRates;  // values taken from http://www.jameco.com/Jameco/Products/ProdDS/2150248.pdf
int hz = 75; //match the switch for hz range
dataRates rate;

float lastRead, lastWriteRead; // Time for getting data from the compass
unsigned long compassError;
int mps = 100; // Max RPM of 55-65 or this will now work, mps is the measure per sec
long rpm_t;
double rpm_m;
boolean rpm_s;

int calibration_cycle = 3; // Value is in minutes
long calibration_time;
int calibration_cycles = 240;

// End of code to check antenna

uint8_t payload[12];
int payload_size = 4;

void calibration()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Starting Calibration."));
  int x, y, z;
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
  int offX = 0;
  int offY = 0;

  for (int i = 0; i < calibration_cycles; i++)
  {
    //   // step 1: instruct sensor to read echoes
    // Wire.beginTransmission(Mag);  // transmit to device
    // // the address specified in the datasheet is 66 (0x42)
    // // but i2c adressing uses the high 7 bits so it's 33
    // Wire.write(0x03);          // command sensor to measure angle
    // Wire.endTransmission();  // stop transmitting

    // The above code is not required when in continuous mode as the sensor only updates at its predefined rate

    // step 3: request reading from sensor
    Wire.requestFrom(Mag, 6);
    if(6 <= Wire.available())     // if two bytes were received
    {
      x = Wire.read()<<8; //X msb
      x |= Wire.read(); //X lsb
      z = Wire.read()<<8; //Z msb
      z |= Wire.read(); //Z lsb
      y = Wire.read()<<8; //Y msb/
      y |= Wire.read(); //Y lsb
    }

    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;

    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;
    delay(1000 / mps);
  }

  xoff = offX;
  yoff = offY;

  lcd.clear();
  lcd.print(F("Finished Calibration"));
}

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(9600);

  Serial.println(F("Initializing sub-systems"));

  // This switch sets the becon to the correct hz for readings
  if (cont)
    switch(hz)
    {
      case 75:
        rate = DATARATE_75HZ;
        break;
      case 30:
        rate = DATARATE_30HZ;
        break;
      case 15:
        rate = DATARATE_15HZ;
        break;
      case 57:
        rate = DATARATE_7_5HZ;
        break;
      case 3:
        rate = DATARATE_3HZ;
        break;
      case 51:
        rate = DATARATE_1_5HZ;
        break;
      case 750:
        rate = DATARATE_0_75_HZ;
        break;
      default:
        rate = DATARATE_15HZ;
    }

  lcd.begin(20, 4);
  lcd.print(F("Starting..."));
  lcd.noAutoscroll();

  Wire.begin();
  xBEE.begin(57600);
  xBee.setSerial(xBEE);

  if (cont)
  {
    // Start Magnetometer
    Wire.beginTransmission(Mag);
    // Tells magnetometer to enable writing
    Wire.write(0x02);
    // tells mag to go to measure mode
    Wire.write(0x00);
    Wire.endTransmission();

    // Change data rate to 75 hz
    Wire.beginTransmission(Mag);
    Wire.write(0x00);
    Wire.endTransmission();

    uint8_t value = 0;

    Wire.beginTransmission(Mag);
    Wire.requestFrom(Mag, 1);
    while(!Wire.available())
      value = Wire.read();
    Wire.endTransmission();
    value &= 0b11100011;
    value |= (rate << 2);

    Wire.beginTransmission(Mag);
    Wire.write(0x00);
    Wire.write(value);
    Wire.endTransmission();

    mps = hz;
    // End of change rate change
  }

  delay(1000);

  // Init compass/antenna calculate RPM

  lastRun = millis();
  lastRead = lastRun;
  lastSend = lastRead;
  rpm_t = lastSend;

  // Enable calibration only when the beacon is not rotating
  //calibration();

  delay(1000);

  lcd.clear();
  lcd.begin(20, 4);
  lcd.println(F("Finished Initialization."));

  Serial.end(); // Comment this line to enable debugging
}
/*--------------------------------------------------------------
This the the function which gathers the heading from the compass.
----------------------------------------------------------------*/
void getVector () {
  float reading = -1;
  int x,y,z;

  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(Mag);  // transmit to device
  // the address specified in the datasheet is 66 (0x42)
  // but i2c adressing uses the high 7 bits so it's 33
  Wire.write(0x03);          // command sensor to measure angle
  Wire.endTransmission();  // stop transmitting

  // The above code is not required when in continuous mode as the sensor only updates at its predefined rate

  // step 3: request reading from sensor
  Wire.requestFrom(Mag, 6);

  // step 4: receive reading from sensor
  if(6 <= Wire.available())     // if two bytes were received
  {
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb/
    y |= Wire.read(); //Y lsb
  }
  //Adjust values by offsets
  x += xoff;
  y += yoff;
  z += zoff;
  //Scale axes
  x *= xscale;
  y *= yscale;
  z *= zscale;

  // Ensure the compass is level for this to work.
  float heading = atan2(y,x);
  //heading += PI/2;  // WHY is this here
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  reading = heading * 180/PI;
  reading += declination;
  if (reading < 0)
    reading += 360;
  heading_converter.f = reading;    // return the heading or bearing
}

void makePayload(){
  memcpy(payload, heading_converter.b, 4);
}

double calculate_RPM(double aOne, double aTwo, double aTime)
{
  Serial.println((String)aOne + " " + (String)aTwo + " " + (String)aTime);
  return abs(aOne - aTwo) * (1000 / (aTime * 6));
}

void loop() {
  // put your main code here, to run repeatedly:
  long time_ = millis();
  if (time_ - lastSend > mps)
  {
    unsigned long start = millis();
    getVector();
    unsigned long now = millis();
    compassError = now - start;
    if (!rpm_s)
    {
      rpm_m = heading_converter.f;
      rpm_t = now;
      rpm_s = true;
    }
    makePayload();
    Tx16Request tx = Tx16Request(0x5678, payload, sizeof(payload));
    xBee.send(tx);
    lastSend = millis();
  }
  if (time_ - lastRun > 250)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print((String)F("Heading: ") + (String)heading_converter.f + char(223));
    lcd.setCursor(0,1);
    lcd.print((String)F("Error: ") + (String)(compassError) + " ms " + F("MPS: ") + mps);
    lcd.setCursor(0,2);
    lcd.print((String)F("RPM: ") + RPM + F(" HZ: ") + (RPM / 60));
    lcd.setCursor(0,3);
    lcd.print((String)F("Paylod: ") + (String)payload[0] + (String)payload[1] + (String)payload[2] + (String)payload[3]);
    lastRun = millis();
    lastWriteRead = heading_converter.f;

    Serial.println(payload[0]);
    Serial.println(payload[1]);
    Serial.println(payload[2]);
    Serial.println(payload[3]);
    Serial.println(payload[4]);
    Serial.println(payload[5]);
    Serial.println(payload[6]);
    Serial.println(payload[7]);
    Serial.println(payload[8]);
    Serial.println(payload[9]);
    Serial.println(payload[10]);
    Serial.println(payload[11]);
    Serial.println("---------------------------");
    Serial.println(heading_converter.b[0]);
    Serial.println(heading_converter.b[1]);
    Serial.println(heading_converter.b[2]);
    Serial.println(heading_converter.b[3]);
    Serial.println("---------------------------");
  }
  if ((time_ - rpm_t) > 500)
  {
    RPM = calculate_RPM(rpm_m, heading_converter.f, (double)abs(millis() - rpm_t));
    rpm_s = false;
  }
}
