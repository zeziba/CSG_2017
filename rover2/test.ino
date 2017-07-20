#include <Servo.h>
//Motor Shield
// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.
#define CW  1
#define CCW 0
#define MOTOR_A 0
#define MOTOR_B 1
const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B
//SONAR
const int pwPin = 0;
//SERVOS
Servo servo1R;
Servo servo2R;
Servo servo1L;
Servo servo2L;

int pos = 0;

uint8_t FWD = 2000;
uint8_t REV = 1000;
uint8_t BRK = 1500;

//Pins 9,10,A = Right Side
//Pins 5,6,B = Left Side

void setup() {
  // put your setup code here, to run once:
  setupArdumoto(); // Set all pins as outputs
  Serial.begin(9600);
//Servo Attachment
  servo1R.attach(6);
  servo1R.writeMicroseconds(BRK);
  servo2R.attach(5);
  servo2R.writeMicroseconds(BRK);
  servo1L.attach(9);
  servo1L.writeMicroseconds(BRK);
  servo2L.attach(10);
  servo2L.writeMicroseconds(BRK);
}

void loop() {
  // put your main code here, to run repeatedly:
forward();
Serial.print("forward");
delay(4000);
reverse;
Serial.print("reverse");
delay(4000);
right;
Serial.print("right");
delay(4000);
left;
Serial.print("left");
delay(4000);
}

// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
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

// stopArdumoto makes a motor stop
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

// setupArdumoto initialize all pins
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

void forward()
{
  driveArdumoto(MOTOR_A, CCW, 255);
  driveArdumoto(MOTOR_B, CCW, 255);
  servo1R.writeMicroseconds(FWD);
  servo2R.writeMicroseconds(FWD);
  servo1L.writeMicroseconds(FWD);
  servo2L.writeMicroseconds(FWD);

}
void reverse()
{
  driveArdumoto(MOTOR_A, CCW, 127);
  driveArdumoto(MOTOR_B, CCW, 127);
  servo1R.writeMicroseconds(REV);
  servo2R.writeMicroseconds(REV);
  servo1L.writeMicroseconds(REV);
  servo2L.writeMicroseconds(REV);

}
void right()
{
  driveArdumoto(MOTOR_A, CCW, 255);
  servo1R.writeMicroseconds(FWD);
  servo2R.writeMicroseconds(FWD);
  servo1L.writeMicroseconds(REV);
  servo2L.writeMicroseconds(REV);
  driveArdumoto(MOTOR_B, CCW, 127);
}
void left(){
  driveArdumoto(MOTOR_A, CCW, 127);
  servo1R.writeMicroseconds(REV);
  servo2R.writeMicroseconds(REV);
  servo1L.writeMicroseconds(FWD);
  servo2L.writeMicroseconds(FWD);
  driveArdumoto(MOTOR_B, CCW, 255);
}
