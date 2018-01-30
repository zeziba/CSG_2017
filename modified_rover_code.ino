#include <Wire.h>
#include <XBee.h>
#include <SoftwareSerial.h>

// PI constant - overrides PI constant
#define M_PI 3.14159265358979311599796346854418516159057617188
// END Pi

// Wire code
bool alive = false;
uint8_t buffer[512];
// END Wire code

//////////////////////////////////
// xBee Code
SoftwareSerial xb = SoftwareSerial(6, 5);

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
  Serial.println("Shutting down wire.");
  Wire.end();
  alive = false;
  Serial.println("Finsihed shutting down wire.");
}

void setupXBEE()
{
  Serial.println("Seting up xBee device.");
  xb.begin(57600);
  xbee.setSerial(xb);
  Serial.println("Finished seting up xBee device.");
}

void suSerial(int baud) {
  Serial.begin(baud);
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
  suSerial(9600);
  delay(250);

  Serial.println("Begining Setup of Device.");

  //setupWire();

  delay(250);
  setupXBEE();
  
  last_check_compass = millis();
  last_check_detection = last_check_compass;

  Serial.println("Finished Setup of Device");
}

void loop()
{
  unsigned long time_ = millis();

  if (time_ - last_check_compass > compass)
  {
    getSamples();
    last_check_compass = millis();
    ProcessData();
  }
}
