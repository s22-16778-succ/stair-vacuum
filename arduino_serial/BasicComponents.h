/*
 * 16-779 Mechatronic Design
 * Team E: Stair Utility Cleaning Contraption
 * Felipe Borja, Jaiden Napier, Ignacio Peon, Sahil Saini, Brandon Wang
 * Created by: Jaiden Napier and Brandon Wang on 4/16/2022
 * Purpose of Code: library header file for initializing and controlling
 *        motors (with L289N motor driver) and ultrasonic sensors.
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
