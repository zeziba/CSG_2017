#include <Arduino.h>
#include <Wire.h>

#define DUE 0x00
#define COMPASS 0x1E
#define FIO 0x64

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
// Code for the Compass
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// PI constant
#define M_PI 3.14159265358979311599796346854418516159057617188

//Axis offsets for magnetometer
int xoff = -7;
int yoff = 54;
int zoff = 0;

//Axis scales for magnetometer
float xscale = 1.070;
float yscale = 1.117;
float zscale = 1;

union{
  float f;
  uint8_t b[4];
}heading_converter; // Current heading

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

int mps = 100; // Max RPM of 55-65 or this will now work, mps is the measure per sec

int calibration_cycle = 3; // Value is in minutes
long calibration_time;
int calibration_cycles = 240;

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
// End of Compass Code
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// Declare Functions
void calibration();
void setRequestSettings();
void getVector();

// lock variable to stop operation of controls
bool lock = false;

void setup() {
  // put your setup code here, to run once:

  delay(1000);

  Wire.begin(); // create as master

  // Ensure that COMPASS is alive
  boolean test = true;
  while (test)
  {
    int error_compass;

    if (error_compass != 0)
    {
      Wire.beginTransmission(COMPASS);
      error_compass = Wire.endTransmission();
    }

    if (error_compass == 0) // a error code of 0 means that there was successful communication
      test = false;
  }

  // Set up the compass
  if (cont)
  {
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
    // Start Magnetometer
    Wire.beginTransmission(COMPASS);
    // Tells magnetometer to enable writing
    Wire.write(0x02);
    // tells mag to go to cont. measure mode
    Wire.write(0x00);
    Wire.endTransmission();

    // Change data rate to Specified hz
    Wire.beginTransmission(COMPASS);
    Wire.write(0x00);
    Wire.endTransmission();

    uint8_t value = 0;

    Wire.beginTransmission(COMPASS);
    Wire.requestFrom(COMPASS, 1);
    while(!Wire.available())
      value = Wire.read();
    Wire.endTransmission();
    value &= 0b11100011;
    value |= (rate << 2);

    Wire.beginTransmission(COMPASS);
    Wire.write(0x00);
    Wire.write(value);
    Wire.endTransmission();

    mps = hz;
    // End of change rate change
  } // the else isimplict to the compass, by not switching the mod it defaults to single measure mode with a maximum of 160hz pooling rate

  delay(100); // wait for compass to get ready

  calibration(); // calibrate the compass

  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void setRequestSettings(byte settings[])
{
  Wire.beginTransmission(FIO);
  for (unsigned int i = 0; i < sizeof(settings) / sizeof(settings[0]); i++)
    Wire.write(settings[i]);
  int error = Wire.endTransmission();
  if (error != 0)
    setRequestSettings(settings);
  Serial.println(F("End of setting FIO"));
}

/*--------------------------------------------------------------
This the the function which gathers the heading from the compass.
----------------------------------------------------------------*/
void getVector () {
  float reading = -1;
  int x,y,z;

  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(COMPASS);  // transmit to device
  // the address specified in the datasheet is 66 (0x42)
  // but i2c adressing uses the high 7 bits so it's 33
  Wire.write(0x03);          // command sensor to measure angle
  Wire.endTransmission();  // stop transmitting

  // The above code is not required when in continuous mode as the sensor only updates at its predefined rate

  // step 3: request reading from sensor
  Wire.requestFrom(COMPASS, 6);

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
  heading += PI/2;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  reading = heading * 180/PI;
  heading_converter.f = reading;    // return the heading or bearing
}


void calibration()
{
  lock = true;
  Serial.println(F("Starting Calibration."));
  int x, y, z;
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
  int offX = 0;
  int offY = 0;

  for (int i = 0; i < calibration_cycles; i++)
  {
      // step 1: instruct sensor to read echoes
    Wire.beginTransmission(COMPASS);  // transmit to device
    // the address specified in the datasheet is 66 (0x42)
    // but i2c adressing uses the high 7 bits so it's 33
    Wire.write(0x03);          // command sensor to measure angle
    Wire.endTransmission();  // stop transmitting

    // The above code is not required when in continuous mode as the sensor only updates at its predefined rate

    // step 3: request reading from sensor
    Wire.requestFrom(COMPASS, 6);
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

  Serial.println(F("Finished Calibration"));
  lock = false;
}
