/*
 * 16-779 Mechatronic Design
 * Team E: Stair Utility Cleaning Contraption
 * Felipe Borja, Jaiden Napier, Ignacio Peon, Sahil Saini, Brandon Wang
 * Created by: Brandon Wang on 4/16/2022
 * Purpose of Code: library functions for initializing and controlling
 *        motors (with L289N motor driver) and ultrasonic sensors.
 */

#include "Arduino.h"
#include "BasicComponents.h"

// DC motors, no encoders
Motor::Motor(int in1Pin, int in2Pin, int enPin)
{
  in1 = in1Pin;
  in2 = in2Pin;
  en = enPin;
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}
// DC motors with encoders
Motor::Motor(int in1Pin, int in2Pin, int enPin, int encaPin, int encbPin)
{
  in1 = in1Pin;
  in2 = in2Pin;
  en = enPin;
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  enca = encaPin;
  encb = encbPin;
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);

  pos = 0;
  target = 0;
  reached = 0;
}

void Motor::setSpeed(int Speed)
{
  if (Speed == 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  else if (Speed > 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en,max(min(Speed,255),0));
  }
  else if (Speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en,max(min(-Speed,255),0));
  }

}


// Ultrasonic Sensor
Ultrasonic::Ultrasonic(int trigPin, int echoPin)
{
  trig = trigPin;
  echo = echoPin;
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

// distance is in inches
double Ultrasonic::get_distance()
{
  // Trigger
  digitalWrite(trig, LOW);  delayMicroseconds(5);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Echo
  double duration = pulseIn(echo, HIGH);
  delay(50); // extra delay to prevent reading too quickly
  return min(duration * SOUND_SPEED / 2, 100.0);
}
