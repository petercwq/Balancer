
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

float ms = 1;

// initialize the stepper library on pins 8 through 11:
Stepper stepperL(200, 4, 5, 6, 7);
Stepper stepperR(200, 8, 9, 10, 12);

void setup()
{
  // initialize the serial port:
  Serial.begin(115200);

  stepperL.setSpeed(400);
  stepperL.setSpeed(400);
}

void loop()
{
  if (Serial.available())
  {
    byte r = Serial.read();
    if (r == '-' || r == '=')
    {
      float flag = 1;
      if (r == '-')
        flag = -1;
      if (ms > 0.05)
        ms += 0.05 * flag;
      else
        ms += 0.0005 * flag;
      Serial.println(ms*1000);
    }
  }

  stepperL.step(1);
  stepperR.step(1);

  if(ms > 0.005)
  {
    delay((unsigned int)(ms * 1000));
  }
  else 
  {
    delayMicroseconds((unsigned int)(ms * 1000000));
  }
}
