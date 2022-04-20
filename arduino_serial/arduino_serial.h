// I2C for IMU
#include "ICM_20948.h"
#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM;

// Commands from Raspi
String strings[10];
char command[100];
char *ptr = NULL;

// Ultrasonic Sensors: [TRIG, ECHO] for FRONT_LEFT, FRONT_RIGHT, LEFT, RIGHT
int ALL_US[4][2] = {{52,50}, {53,51}, {49,47}, {48,46}};
float SOUND_SPEED = 0.0135039; // inches per microsecond

// Bumpers
int ALL_BUMPERS[2] = {2, 3};
int DEBOUNCE_DELAY = 50; // ms
int lastDebounce = 0;
int bumperState = 0; // 0=nothing, 1=left, 2=right

// Buzzer
int BUZZER = 4;

// Wheels/DC Motors: [OUT1, OUT2, EN] for FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
// EN must be PWM!
int MOTORS[4][3] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};

// Vacuum Motor: [OUT1, OUT2]
int VACUUM[2] = {0,0};

// Lift Motors: [OUT1, OUT2, PWM, ENCA, ENCB, pos] for FRONT_LIFT, BACK_LIFT
int LIFTS[2][6] = {{0,0,0,0,0,0}, {0,0,0,0,0,0}};

// Global state
String POSITION;
int STEPS = 8;
int PASSES = 0;
int ON_PIN = 0; // this will be a button
