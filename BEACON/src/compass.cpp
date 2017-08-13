//
// Created by cengen on 8/13/17.
//

#include <Wire.h>
// Address location for magnetometer
// The manometer can only communicate at 160hz max, this translates into a 0.00625 second delay between each signal
// This translates into a max of 160 readings per second
#define Mag 0x1E

double declination = -8.51;  // Declination around Pueblo Colorado


//Axis offsets for magnetometer
int xoff = -7;
int yoff = 54;
int zoff = 0;

//Axis scales for magnetometer
float xscale = 1.070;
float yscale = 1.117;
float zscale = 1;

// Struct used as a data storage device
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

int calibration_cycle = 3; // Value is in minutes
long calibration_time;
int calibration_cycles = 240;

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
