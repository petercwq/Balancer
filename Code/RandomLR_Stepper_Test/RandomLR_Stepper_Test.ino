// Random.pde
// -*- mode: C++ -*-
//
// Make a single stepper perform random changes in speed, position and acceleration
//
// Copyright (C) 2009 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#include "AccelStepper.h"

// Define a stepper and the pins it will use
AccelStepper stepperL(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper stepperR(AccelStepper::FULL4WIRE, 6, 7, 8, 9);

void setup()
{  
}

void loop()
{
    if (stepperL.distanceToGo() == 0)
    {
        // Random change to speed, position and acceleration
        // Make sure we dont get 0 speed or accelerations
        delay(1000);
        stepperL.moveTo(rand() % 200);
        stepperL.setMaxSpeed((rand() % 200) + 1);
        stepperL.setAcceleration((rand() % 200) + 1);
    }

    if (stepperR.distanceToGo() == 0)
    {
        // Random change to speed, position and acceleration
        // Make sure we dont get 0 speed or accelerations
        delay(1000);
        stepperR.moveTo(rand() % 200);
        stepperR.setMaxSpeed((rand() % 200) + 1);
        stepperR.setAcceleration((rand() % 200) + 1);
    }

    stepperL.run();
    stepperR.run();
}