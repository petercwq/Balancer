
/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 */

#include <Stepper.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper1(stepsPerRevolution, 6, 7, 8, 9);
Stepper myStepper2(stepsPerRevolution, 2, 3, 4, 5);


void setup() {
  // initialize the serial port:
  Serial.begin(115200);
}

void loop() {
  unsigned long t = micros();
  myStepper1.step(1);
  myStepper2.step(1);
  Serial.println(micros()-t);
  delay(2);
}

