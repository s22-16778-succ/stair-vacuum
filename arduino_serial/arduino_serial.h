// I2C for IMU
#include "ICM_20948.h"
#include <Motor.h>
#include <Servo.h>
#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM;

// Commands from Raspi
String strings[10];
char command[100];
char *ptr = NULL;

// ON switch
int ON_SWITCH = 12;

// Ultrasonic Sensors: [TRIG, ECHO] for FRONT_LEFT, FRONT_RIGHT, LEFT, RIGHT
int US[4][2] = {{52,53}, {50,51}, {48,49}, {46,47}}; // COMPONENTS ORDER MAY BE WRONG!!!
float SOUND_SPEED = 0.0135039; // inches per microsecond

// Bumpers
int BUMPERS[2] = {28,29}; // COMPONENTS ORDER MAY BE WRONG!!!

// Limit Switches
int LIMITS[2] = {18,19}; // CURRENTLY UNUSED!!!!
int DEBOUNCE_DELAY = 50; // ms
int lastDebounce = 0;
int limitState[2] = {0,0}; // left, right; 0=pressed, 1=off stair

// Buzzer
int BUZZER = 11;
int BUZZER_FREQ = 1100;

// Wheels/DC Motors: [IN1, IN2, EN] for FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
// EN must be PWM!
int MOTORS[4][3] = {{36,37,9}, {34,35,8}, {32,33,7}, {30,31,6}}; // COMPONENTS ORDER MAY BE WRONG!!!
int MOTOR_SPD = 120; // default motor speed
int MOTOR_DEL =  10; // default motor delay (ms) (step size)

// Vacuum Motor
int VACUUM = 10;
int MIN_PULSE = 1000;
int MAX_PULSE = 2000;
Servo ESC;     // create servo object to control the ESC

// Lift Motors: [IN1, IN2, EN, ENCA, ENCB] for FRONT_LIFT, BACK_LIFT
int LIFTS[2][6] = {{42,43,4,2,45}, {38,39,5,3,41}};
// [pos, target, reached, dir] for FRONT_LIFT, BACK_LIFT
int lift_info[2][4] = {{0,0,0,0}, {0,0,0,0}};
int LIFT_SPD[2] = {145, 120}; // default motor speed

// Global state
String POSITION;
int STEPS = 8;
int ON_PIN = 0; // this will be a button
