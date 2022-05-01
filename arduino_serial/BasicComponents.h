/*
  BasicComponents.h - Library for using Motors (with L289N motor driver)
  Created by Brandon Wang and Jaiden Napier April 16 2022.
  Released into the public domain.
*/
#ifndef BasicComponents_h
#define BasicComponents_h

#include "Arduino.h"
class Motor
{
  public:
    // Functions
    Motor(int in1Pin, int in2Pin, int enPin);
    Motor(int in1Pin, int in2Pin, int enPin, int encaPin, int encbPin);
    void setSpeed(int Speed);

    // Pins
    int in1, in2, en, enca, encb;

    // Stored Encoder info
    int pos;
    int target;
    int reached;

};

class Ultrasonic
{
  public:
    // Functions
    Ultrasonic(int trigPin, int echoPin);
    double get_distance();

    // Pins
    int trig, echo;

    // Ultrasonic Values
    int SOUND_SPEED = 0.0135039; // inches per microsecond

};

#endif