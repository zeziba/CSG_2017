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
#include "compass.h"
#include <XBee.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Arduino.h>

// PI constant
#define M_PI 3.14159265358979311599796346854418516159057617188


// Starts the lcd display pinout
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// The below Code will be used to calculate the amount of time that is required for each read of the magometer and when it sends the data
float RPM = 1.0;

SoftwareSerial xBEE(2, 3); // TX, RX lines
XBee xBee = XBee();
int resetRSSI = -1000; // Reset RSSI number

// Variables used as timers
double lastRun; // Variable to indicate last time run
double lastSend; // Varialbe to indicate last sent bytes

float lastRead, lastWriteRead; // Time for getting data from the compass
unsigned long compassError;
int mps = 100; // Max RPM of 55-65 or this will not work, mps is the measure per sec
long rpm_t;
double rpm_m;
boolean rpm_s;

// End of code to check antenna

uint8_t payload[12];
int payload_size = 4;

// Struct used as a data storage device
union{
    float f;
    uint8_t b[4];
}heading_converter; // Current heading

bool debug = true;


void setup() {
    // put your setup code here, to run once:
    delay(1000);
    if (debug)
        Serial.begin(9600);

    Serial.println(F("Initializing sub-systems"));

    if (cont) // cont is a value set in the compass, it indicates if continuous mode is used
        set_hz(hz);

    lcd.begin(20, 4);
    lcd.print(F("Starting..."));
    lcd.noAutoscroll();

    Wire.begin();
    xBEE.begin(57600);  // setting given by the csg in order to emulate the beacon
    xBee.setSerial(xBEE);

    if (cont)
        setup_hz_rate();

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
        lcd.print((String)F("Payload: ") + (String)payload[0] + (String)payload[1] + (String)payload[2] + (String)payload[3]);
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
