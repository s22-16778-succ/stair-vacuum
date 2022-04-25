#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int p1,int p2, int ep)
{
 pin1 = p1;
 pin2 = p2;
 enaPin = ep;
 pinMode(enaPin, OUTPUT);
 pinMode(pin1, OUTPUT);
 pinMode(pin2, OUTPUT);
}

void Motor::setDir(int dir) 
{
  if (dir>0){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else{
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
}

void Motor::motorSpeed(int Speed)
{
  analogWrite(enaPin,max(min(Speed,255),0));
}
