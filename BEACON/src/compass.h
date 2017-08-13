//
// Created by cengen on 8/13/17.
//
#include "compass.cpp"

#ifndef CSG_2017_COMPASS_H

void set_hz(double hz) {
    // TODO: Clean the switch up so it better matches a readable value
    switch((int)(hz * 100))
    {
        case 7500:
            rate = DATARATE_75HZ;
            break;
        case 3000:
            rate = DATARATE_30HZ;
            break;
        case 1500:
            rate = DATARATE_15HZ;
            break;
        case 750:
            rate = DATARATE_7_5HZ;
            break;
        case 300:
            rate = DATARATE_3HZ;
            break;
        case 150:
            rate = DATARATE_1_5HZ;
            break;
        case 75:
            rate = DATARATE_0_75_HZ;
            break;
        default:
            rate = DATARATE_15HZ;
    }
}

void setup_hz_rate() {
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



#define CSG_2017_COMPASS_H

#endif //CSG_2017_COMPASS_H
