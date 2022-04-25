#include "Arduino.h"
#include "Motor.h"

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
  Motor(in1Pin, in2Pin, enPin);
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
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en,max(min(Speed,255),0));
  }
  else if (Speed < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en,max(min(-Speed,255),0));
  }

}