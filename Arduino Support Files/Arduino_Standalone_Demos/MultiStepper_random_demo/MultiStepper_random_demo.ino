// MultiStepper_random_demo.ino
// -*- mode: C++ -*-
//
// Shows how to multiple simultaneous steppers
// Runs one stepper forwards and backwards, accelerating and decelerating
// at the limits. Runs other steppers at the same time
//
// Copyright (C) 2022 Jonathan Shulgach


// This sketch sets the position for the stepper motors to run to as opposed to setting a speed control.
// Doing the speed control blocks the motors until the position has been reached. Doing it
// this way allows the stepper motors to run independently of each other using the Stepper 
// library

#include <AccelStepper.h>

#define motorInterfaceType 1
#define enablePin 30
#define enablePin2 24
//#define enablePin3 56
#define enablePin3 38

// Define some steppers and the pins the will use to test
//AccelStepper stepper1; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper1(motorInterfaceType, 36, 34); // grippwr bend  (joint4)
AccelStepper stepper2(motorInterfaceType, 26, 28); // wrist roll    (joint3)
//AccelStepper stepper3(motorInterfaceType, 60, 61); // rotation base (joint0)
AccelStepper stepper3(motorInterfaceType, 54, 54); // rotation base (joint0)

void setup()
{  
    pinMode(enablePin, OUTPUT);   
    digitalWrite(enablePin, LOW);
    
    pinMode(enablePin2, OUTPUT);   
    digitalWrite(enablePin2, LOW);

    pinMode(enablePin3, OUTPUT);   
    digitalWrite(enablePin3, LOW);

    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(1000.0);
    stepper1.moveTo(1000);
    
    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(1000.0);
    stepper2.moveTo(500);
    
    stepper3.setMaxSpeed(100.0);
    stepper3.setAcceleration(100.0);
    stepper3.moveTo(500); 
}

void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0){
      stepper1.moveTo(-stepper1.currentPosition());
    }
    if (stepper2.distanceToGo() == 0){
      stepper2.moveTo(-stepper2.currentPosition());
    }
    if (stepper3.distanceToGo() == 0){
      stepper3.moveTo(-stepper3.currentPosition());
    }
    stepper1.run();
    stepper2.run();
    stepper3.run();
}
