/*
  Motor.h - Library for using L289N motor driver
  Created by Jaiden Napier April 16 2022.
  Released into the public domain.
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
class Motor
{
  public:
    // Functions
    Motor(int in1Pin, int in2Pin, int enPin);
    Motor(int in1Pin, int in2Pin, int enPin, int encaPin, int encbPin);
    void setSpeed(int Speed);

    // Pins
    int in1;
    int in2;
    int en;
    int enca;
    int encb;

    // Stored Encoder info
    int pos;
    int target;
    int reached;

 };

 #endif
