// I2C for IMU
#include "ICM_20948.h"
#include <Motor.h>
#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM;

// Commands from Raspi
String strings[10];
char command[100];
char *ptr = NULL;

// Ultrasonic Sensors: [TRIG, ECHO] for FRONT_LEFT, FRONT_RIGHT, LEFT, RIGHT
int US[4][2] = {{52,51}, {50,49}, {48,47}, {46,45}}; // COMPONENTS ORDER MAY BE WRONG!!!
float SOUND_SPEED = 0.0135039; // inches per microsecond

// Bumpers
int BUMPERS[2] = {28, 27}; // COMPONENTS ORDER MAY BE WRONG!!!

// Limit Switches
int LIMITS[2] = {18,19}; // CURRENTLY UNUSED!!!!
int DEBOUNCE_DELAY = 50; // ms
int lastDebounce = 0;
int limitState[2] = [0,0]; // left, right; 0=pressed, 1=off stair

// Buzzer
int BUZZER = 11;

// Wheels/DC Motors: [IN1, IN2, EN] for FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
// EN must be PWM!
int MOTORS[4][3] = {{36,35,9}, {34,33,8}, {32,31,7}, {30,29,6}}; // COMPONENTS ORDER MAY BE WRONG!!!
int MOTOR_SPD = 120; // default motor speed
int MOTOR_DEL =  10; // default motor delay (ms) (step size)

// Vacuum Motor
int VACUUM = 10;

// Lift Motors: [IN1, IN2, EN, ENCA, ENCB] for FRONT_LIFT, BACK_LIFT
int LIFTS[2][6] = {{38,37,5,3,39}, {42,41,4,2,43}}; // COMPONENTS ORDER MAY BE WRONG!!!
// [pos, target, reached] for FRONT_LIFT, BACK_LIFT
int lift_info[2][3] = {{0,0,0}, {0,0,0}};

// Global state
String POSITION;
int STEPS = 8;
int PASSES = 0;
int ON_PIN = 0; // this will be a button
