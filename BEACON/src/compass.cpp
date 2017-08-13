//
// Created by cengen on 8/13/17.
//

#include <Wire.h>
// Address location for magnetometer
// The manometer can only communicate at 160hz max, this translates into a 0.00625 second delay between each signal
// the max hz can only be achieved with complex setup and the results are not very accurate. The max supported speed is
// 75 hz
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