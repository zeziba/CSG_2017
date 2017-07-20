/* ////////////////////////////////
This sketch finds the heading (degrees) of the transmitting beacon relative to
the receiver. For example, if the receiver is due west of the beacon, this sketch
will return a heading of 90 degrees.
This sketch receives a packet and stores the RSSI (signal strength) in the RSSIArray
with an index of the Heading (data). It does so 'Samples' amount of times. Then the data
is passed through a digital filter. If the RSSI was not evaluated for a specific heading,
then that data point is not evaluated. This prevents values that were not measured from
effecting the output of the digital filter.
Authored By: Adam St. Amand
Modified by Robert Belter 10/30/2015
- Improved calculation of heading. Now uses floating point value from beacon and does circular mean averaging
- Added support for i2c communication
//////////////////////////////////*/





#include <XBee.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define self 0xA

SoftwareSerial outputSerial(10, 9); // RX, TX

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();
int resetRSSI = -1000;    //The value that RSSI is reset to after each pass through filter
#define samples 110
int temp, smoothData, rawData;
int timeToScan = 1000;
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
}
heading_converter;
//To find the vector angle to beacon
tolerance = 5;
beacon = currentHeading ();
n = beacon - 180;
if (n < 0 )
{
  n += 360;
}
}
void setup() {
  //Initialize serial communications at 57600 bps:
  Serial.begin(57600);

  //Initialize i2c communications
  Wire.begin(self);                // join i2c bus with address #8
  Wire.onRequest(i2cPrint); // register event

  outputSerial.begin(57600);
  xbee.setSerial(outputSerial);
}

void loop() {
  //Retrieve necessary numbers of samples
  //TODO: Improve delay system
  int start = millis();
  for(int i = 0;i<samples;i++){
    Retrieve(i);
    float propComplete = ((float)i)/(float)samples;
    delay(timeToScan/samples);
  }

  //Process the data, print the result, and reset.
  currentHeading = (ProcessData());
  currHeadingI2c[0] = 0xFF&(currentHeading>>8);
  currHeadingI2c[1] = 0xFF&currentHeading;
  Serial.print("Current Heading is "); Serial.println(currentHeading);

  if abs(currentHeading - n > tolerance)
  {
    servo.writeMicroseconds(turn);
  }
  else
  if (sensor.getDistance < 100)
    {
    servo.writeMicroseconds(turn);
  }
  else
  {
    servo.writeMicroseconds(FWD);
  }
}

/////////////////////////////////////////////////////
////////////////Local Functions//////////////////////
/////////////////////////////////////////////////////


//Receives the transmitted packet and stores the information in RSSIArray.

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
      Serial.print("Readings = ");
      Serial.print(readings[i].heading); Serial.print("    ");
      Serial.println(readings[i].signalStrength);
    }
  }else{
    readings[i].heading = 0;
    readings[i].signalStrength = resetRSSI;
  }
}




//Creates a heading through averaging the readings
int ProcessData(){
  int maxRSSI;
  unsigned long maxIndex = 0;
  maxRSSI = readings[0].signalStrength;

  //Find max RSSI value
  for (int i=1; i < samples; i++) {
    if (maxRSSI < readings[i].signalStrength) {
      maxRSSI = readings[i].signalStrength;
      maxIndex = i;
    }
  }


  //Use the maximum heading to define heading
  float heading = readings[maxIndex].heading;

  //------Averaging Code, currently Defunct-------
  /*

  //Create an average of all the samples
  //Circular mean, so use vector addition
  float headingx = 0;
  float headingy = 0;
  for(int i=1; i< samples; i++){
    //Set magnitude of vector by signal strength
    headingx += readings[i].signalStrength*cos(readings[i].heading*PI/180);
    headingy += readings[i].signalStrength*sin(readings[i].heading*PI/180);
  }

  float heading = atan2(headingy, headingx);
  if(heading < 0)
    heading += 2*PI;
  heading = heading * 180/PI;
  */
  //------End Averaging Code------

  return (int) (heading);    //Return the average of all headings
}

//When info is requested over i2c
void i2cPrint()
{
  Wire.write(currHeadingI2c, 2);
  // as expected by master
}
