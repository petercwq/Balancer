
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
Stepper myStepper1(stepsPerRevolution, 8, 9, 10, 11);
Stepper myStepper2(stepsPerRevolution, 4, 5, 6, 7);


void setup() {
  // set the speed at 60 rpm:
  myStepper1.setSpeed(90);
  myStepper2.setSpeed(90);

  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  // step one revolution  in one direction:
  Serial.println("clockwise");
  myStepper1.step(stepsPerRevolution*90);
  myStepper2.step(stepsPerRevolution*90);
  delay(500);

  // step one revolution in the other direction:
  Serial.println("counterclockwise");
  myStepper1.step(-stepsPerRevolution);
  myStepper2.step(-stepsPerRevolution);
  delay(500);
}

