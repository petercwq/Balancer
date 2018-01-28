// Random.pde
// -*- mode: C++ -*-
//
// Make a single stepper perform random changes in speed, position and acceleration
//
// Copyright (C) 2009 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#include "AccelStepper.h"
#include "MultiStepper.h"

// Define a stepper and the pins it will use
AccelStepper stepperL(AccelStepper::FULL4WIRE, 4, 5, 6, 7);
AccelStepper stepperR(AccelStepper::FULL4WIRE, 8, 9, 10, 12);
MultiStepper steppers;

void setup()
{  
    Serial.begin(9600);
    // Configure each stepper
    stepperL.setMaxSpeed(300);
    stepperR.setMaxSpeed(300);
    // Then give them to MultiStepper to manage
    steppers.addStepper(stepperL);
    steppers.addStepper(stepperR);

    // stepperL.setMaxSpeed(1000);
    // stepperR.setMaxSpeed(1000);

    // stepperL.setSpeed(200);
    // stepperR.setSpeed(200);

    // for(int i=0;i<1000;i++)
    // {
    //     stepperL.runSpeed();
    //     stepperR.runSpeed();
    //     delay(1);
    // }

    // stepperL.setSpeed(350);
    // stepperR.setSpeed(350);

    // stepperL.move(15000);
    // stepperL.setAcceleration(100);

    // stepperR.move(15000);
    // stepperR.setAcceleration(100);

    // while(stepperL.distanceToGo()!=0 || stepperL.distanceToGo()!=0)
    // {
    //     if(stepperL.speed()>1000)
    //     {
    //         stepperL.setSpeed(800);
    //         stepperL.runSpeed();
    //     }
    //     else
    //     {
    //         stepperL.run();
    //     }

    //     if(stepperR.speed()>1000)
    //     {
    //         stepperR.setSpeed(800);
    //         stepperR.runSpeed();
    //     }
    //     else
    //     {
    //         stepperR.run();
    //     }
    // }
}

void loop()
{
    long positions[2] = {stepperL.currentPosition()+ rand()%1000+100, stepperR.currentPosition()+ rand()%1000+100}; // Array of desired stepper positions
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();

    // stepperL.runSpeed();
    // stepperR.runSpeed();

    // if (stepperL.distanceToGo() == 0)
    // {
    //     // Random change to speed, position and acceleration
    //     // Make sure we dont get 0 speed or accelerations
    //     delay(1000);
    //     stepperL.moveTo(rand() % 2000);
    //     stepperL.setMaxSpeed((rand() % 500) + 1);
    //     stepperL.setAcceleration((rand() % 100) + 1);
    // }

    // if (stepperR.distanceToGo() == 0)
    // {
    //     // Random change to speed, position and acceleration
    //     // Make sure we dont get 0 speed or accelerations
    //     delay(1000);
    //     stepperR.moveTo(rand() % 2000);
    //     stepperR.setMaxSpeed((rand() % 500) + 1);
    //     stepperR.setAcceleration((rand() % 100) + 1);
    // }

    // stepperL.run();
    // stepperR.run();
}