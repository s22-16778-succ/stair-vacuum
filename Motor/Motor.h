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
    Motor (int p1,int p2, int ep);
    void setDir(int dir);
    void motorSpeed(int Speed);

  private:
    int pin1;
    int pin2;
    int enaPin;
 };

 #endif
