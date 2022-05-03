#include "BasicComponents.h"

// I2C for IMU
#include "ICM_20948.h"
#define WIRE_PORT Wire
ICM_20948_I2C myICM;
int AD0_VAL = 1;

// Commands from Raspi
String strings[10];
char command[100];
char *ptr = NULL;

// ON switch
int ON_SWITCH = 27;  // LOW (|) means start, HIGH (O) means wait
int POS_SWITCH = 26; // LOW (|) means TOP, HIGH (O) means BOTTOM

// Ultrasonic Sensors: [TRIG, ECHO] for FRONT_LEFT, FRONT_RIGHT, LEFT, RIGHT
int US[4][2] = {{52,53}, {50,51}, {48,49}, {46,47}};
float SOUND_SPEED = 0.0135039; // inches per microsecond

// Bumpers: LEFT and RIGHT
int BUMPERS[2] = {28,29};

// Limit Switches: LEFT and RIGHT
int LIMITS[2] = {18,19}; // CURRENTLY UNUSED!!!!
int DEBOUNCE_DELAY = 50; // ms
int lastDebounce = 0;
int limitState[2] = {0,0}; // left, right; 0=pressed, 1=off stair

// Buzzer
int BUZZER = 11;
int BUZZER_FREQ = 1100;

// Wheels/DC Motors: [IN1, IN2, EN] for FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
int MOTORS[4][3] = {{36,37,9}, {30,31,6}, {34,35,8}, {32,33,7}};
int MOTOR_SPD = 120; // default motor speeds
//int MOTOR_SPDS[4] = {175,175,175,175}; // default motor speeds per motor (runs very well at same speeds!)
int MOTOR_SPDS[4] = {178,178,175,175}; // default motor speeds per motor (runs very well at same speeds!)
int MOTOR_DEL =  10; // default motor delay (ms) (step size)

// Vacuum Motor
#include <Servo.h>
int VACUUM = 10;
int MIN_PULSE = 1000;
int MAX_PULSE = 2000;
Servo ESC;     // create servo object to control the ESC

// Lift Motors: [IN1, IN2, EN, ENCA, ENCB] for FRONT_LIFT, BACK_LIFT
int LIFTS[2][6] = {{42,43,4,2,45}, {38,39,5,3,41}};
int LIFT_SPD[2] = {145, 145}; // default lift motor speeds (front is slightly faster)
int LIFT_TARGET = 4350; // encoder targets for "DOWN"; should be same for both lifts

// Global state
String POSITION;
int STEPS  = 8;
