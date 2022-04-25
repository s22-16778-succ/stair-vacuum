#include <Motor.h> 

#define NMOTORS 2
const int ENCA[] = {2,3}; // pins for front and back motor 
const int ENCB[] = {40,41};// pins for front and back motor
Motor motors[] = {Motor(43,42,4), Motor(38,39,5)};
int x = 0;
double posi[] = {0,0};// encoder position for back and front lift respectively
double pwm[] = {0,0};
int dir[] = {1,1};
double target[] = {-100,10};//target postions
int MoveFlag[] = {0,0};
int MotorsMoved = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA[1],INPUT);
  pinMode(ENCB[1],INPUT);
  pinMode(ENCA[2],INPUT);
  pinMode(ENCB[2],INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]),readEncoder<1>,RISING);

}

void loop() {
  if (x == 0){
     Lift();
     x+=1;
    }
}

template <int j>
void readEncoder(){
  int b = digitalRead(ENCB[j]);
  Serial.println(posi[0]);
  if (j == 1){
    b = -1*b;
  }
  if(b <=0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
 
  if (target[j] == posi[j]){
      MoveFlag[j] = 1;
      motors[j].motorSpeed(0);
  } 
 }

void Lift(){
  pwm[0]=100;
  pwm[1]=100;+
   for(int k = 0; k < NMOTORS; k++)
  {
    dir[k] = ((target[k]-posi[k]) > 0) - ((target[k]-posi[k]) < 0);
    motors[k].setDir(dir[k]);
    motors[k].motorSpeed(pwm[k]);
  }
   while (MotorsMoved!= NMOTORS){
   MotorsMoved = 0;
   for (int k=0; k< NMOTORS; k++){
       MotorsMoved += MoveFlag[k];
   }
  }
   
  }
