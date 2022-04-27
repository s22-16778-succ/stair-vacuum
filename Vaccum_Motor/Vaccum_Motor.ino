/*
        Arduino Brushless Motor Control
     by Dejan, https://howtomechatronics.com
*/

#include <Servo.h>
int x = 0;
Servo ESC;     // create servo object to control the ESC

int potValue;  // value from the analog pin

void setup() {
  Serial.begin(9600);
  // Attach the ESC on pin 9
  ESC.attach(10,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC.write(0); 
  delay(4000); 
 
}

void loop() {
  
  //potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
  //potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  //Serial.println(potValue);
  if (x==0){
  int a[4] = {2000, 1800, 1600, 1400};
  for (int i=0; i<4; i++) {
   ESC.writeMicroseconds(2000);    // Send the signal to the ESC 
   // Serial.println(a[i]);
   delay(1000);
   ESC.write(0); 
    x+=1;
  }
  }
  

}
