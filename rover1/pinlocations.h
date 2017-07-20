#include <Arduino.h>

#define SCL 21
#define SDA 20

#define S0TX 1
#define S0RX 0
#define S1TX 19
#define S1RX 18
#define S2TX 16
#define S2RX 17
#define S3TX 14
#define S3RX 15

#define PMW0 2
#define PMW1 3
#define PMW2 4
#define PMW3 5
#define PMW4 6
#define PMW5 7
#define PMW6 8
#define PMW7 9
#define PMW8 10
#define PMW9 11
#define PMW10 12
#define PMW11 13

#define A0 A0 // PIN 54
#define A1 A1 // PIN 55
#define A2 A2 // PIN 56
#define A3 A3 // PIN 57
#define A4 A4 // PIN 58
#define A5 A5 // PIN 59
#define A6 A6 // PIN 60
#define A7 A7 // PIN 61
#define A8 A8 // PIN 62
#define A9 A9 // PIN 63
#define A10 A10 // PIN 64
#define A11 A11 // PIN 65

#define DAC0 66 // dac pins have true analog output of 4096 levels
#define DAC1 67 // this allows for true audio output, limited to 0.55 - 2.75v

#define LED 13
#define LEDRX 72
#define LEDTX 73

#define I2CPOWER 0

#define PRESSTEMP 0x60
#define TEMPHUMIDITY 0x40
#define XBEETX 21
#define XBEERX 22
#define HMC5883L_ 0x1E

#define CHIPSELECT 4

// Changes pins for SD as needed!
#define MOSI 1
#define MISO 2
#define CS 3
#define SCK 4

#define SONARNUM 1
#define SONAR1TRIGGER 41
#define SONAR1ECHO 42
#define SONAR1MAXDIST 500
