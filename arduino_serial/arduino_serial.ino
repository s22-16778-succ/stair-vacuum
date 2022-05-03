#include "arduino_serial.h"

Motor all_motors[] = { Motor(MOTORS[0][0],MOTORS[0][1],MOTORS[0][2]),
                       Motor(MOTORS[1][0],MOTORS[1][1],MOTORS[1][2]),
                       Motor(MOTORS[2][0],MOTORS[2][1],MOTORS[2][2]),
                       Motor(MOTORS[3][0],MOTORS[3][1],MOTORS[3][2]) };

Motor all_lifts[] = {  Motor(LIFTS[0][0],LIFTS[0][1],LIFTS[0][2],LIFTS[0][3],LIFTS[0][4]),
                       Motor(LIFTS[1][0],LIFTS[1][1],LIFTS[1][2],LIFTS[1][3],LIFTS[1][4]) };

Ultrasonic all_US[] = {  Ultrasonic(US[0][0], US[0][1]),
                         Ultrasonic(US[1][0], US[1][1]),
                         Ultrasonic(US[2][0], US[2][1]),
                         Ultrasonic(US[3][0], US[3][1]) };

void setup() {
  Serial.begin(115200);
  Serial.println("Beginning Setup");

  // On switch (for beginning operation)
  pinMode(ON_SWITCH, INPUT);
  pinMode(POS_SWITCH, INPUT);
  
  // Ultrasonic Sensors
  for (int i=0; i<4; i++) {
    pinMode(US[i][0], OUTPUT);
    pinMode(US[i][1], INPUT);
  }

  // Bumpers
  pinMode(BUMPERS[0], INPUT);
  pinMode(BUMPERS[1], INPUT);

  // Limit Switches
  limitState[0]=0; limitState[1]=0;
  pinMode(LIMITS[0], INPUT_PULLUP);
  pinMode(LIMITS[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMITS[0]), readLimit<0>, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMITS[1]), readLimit<1>, FALLING);

  // Buzzer
  pinMode(BUZZER, OUTPUT);
  buzz(); // "Beginning setup"

  // Wheel motors; set up with Motors.h

  // Vacuum; BLDC
  ESC.attach(VACUUM, MIN_PULSE, MAX_PULSE); // (pin, min pulse width, max pulse width)
  ESC.write(0);
  // delay(4000); // is this delay necessary?

  // Lift Motors; set up with Motors.h
  attachInterrupt(digitalPinToInterrupt(all_lifts[0].enca), readLift<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(all_lifts[1].enca), readLift<1>, RISING);

  // IMU setup
//  IMU_setup(); // Comment/uncomment this !!!

  // Run the code!
  while(digitalRead(ON_SWITCH)) ;
  Serial.println("Beginning code");
  code();
}


void loop() {
//  if (Serial.available() > 0) {
//    // Read in Raspberry Pi command
//    Serial.readStringUntil('\n').toCharArray(command, 100);
//    ptr = strtok(command, ",");
//    int index = 0;
//    while (ptr != NULL) {
//      strings[index++] = ptr;
//      ptr = strtok(NULL, ",");
//    }
//
//    // Interpret the strings
//    // for (int i=0; i<index; i++)
//    //  Serial.println(strings[i]);
//    
//    if (strings[0].equals("get_US"))
//      get_US(strings[1]);
//    if (strings[0].equals("buzz"))
//      buzz();
//
//  }
  
}

double get_US(String sensor) {
  if      (sensor.equals("FRONT_LEFT"))  return get_distance(US[0]);
  else if (sensor.equals("FRONT_RIGHT")) return get_distance(US[1]);
  else if (sensor.equals("LEFT"))        return get_distance(US[2]);
  else if (sensor.equals("RIGHT"))       return get_distance(US[3]);
}
double get_distance(int U[]) {
  // Trigger
  digitalWrite(U[0], LOW);  delayMicroseconds(5);
  digitalWrite(U[0], HIGH); delayMicroseconds(10);
  digitalWrite(U[0], LOW);

  // Echo
  double duration = pulseIn(U[1], HIGH);
  delay(50);
  return min(duration * SOUND_SPEED / 2, 100); // inches
}
double front_US_within(int dist) {
  return (get_US("FRONT_LEFT")<dist && get_US("FRONT_RIGHT")<dist);
}

void buzz() {
  tone(BUZZER, BUZZER_FREQ); delay(200);
  noTone(BUZZER);            delay(200);
}

// Run the front left motor, front right motor, back left motor, and back right motor
// Move/Rotate robot forever (speed is positive)
void move(String dir) { // default speeds
  move(dir, MOTOR_SPDS[0], MOTOR_SPDS[1], MOTOR_SPDS[2], MOTOR_SPDS[3]);
}
// Move wheels at set speed
void move(String dir, int spd) {
  move(dir, spd, spd, spd, spd);
}
// Move/Rotate robot for delay seconds (speed is positive)
void move(String dir, int spd, double del) {
  move(dir, spd);
  delay(del);
  move("OFF",0);
}
// Move each wheel separately
void move(String dir, int spd0, int spd1, int spd2, int spd3) {
  if      (dir.equals("OFF"))      motors(0,0,0,0);
  else if (dir.equals("FORWARD"))  motors( spd0,  spd1,  spd2,  spd3);
  else if (dir.equals("BACKWARD")) motors(-spd0, -spd1, -spd2, -spd3);
  else if (dir.equals("LEFT"))     motors(-spd0,  spd1,  spd2, -spd3);
  else if (dir.equals("RIGHT"))    motors( spd0, -spd1, -spd2,  spd3);
  else if (dir.equals("CW"))       motors( spd0, -spd1,  spd2, -spd3);
  else if (dir.equals("CCW"))      motors(-spd0,  spd1, -spd2,  spd3);
}
// speed is -255 to 255
void motors(double spd0, double spd1, double spd2, double spd3) {
  all_motors[0].setSpeed(spd0);
  all_motors[1].setSpeed(spd1);
  all_motors[2].setSpeed(spd2);
  all_motors[3].setSpeed(spd3);
}

// Read limit switches
template <int lim>
void readLimit() {
  if (limitState[lim] == 0)
    limitState[lim] = 1;
}

// Turn vacuum ON or OFF
void vacuum(String state) {
  if      (state.equals("ON"))  ESC.writeMicroseconds(MAX_PULSE);
  else if (state.equals("OFF")) ESC.write(0);
}
void sweep(String state) {
  double thr=5;
  vacuum("ON");

  move(state, MOTOR_SPD, MOTOR_DEL);
  while(true) {
    if (state.equals("LEFT")  && digitalRead(BUMPERS[0]) == LOW) break;
    if (state.equals("RIGHT") && digitalRead(BUMPERS[1]) == LOW) break;
  }
  
  vacuum("OFF");
  move("OFF",0);
}

void lifts(String state) {

  // Call top (initial) position pos=0 and bottom position pos=LIFT_TARGET
  int target = (state.equals("UP")) ? 0 : LIFT_TARGET;
  if (state.equals("DOWN")) { // NEED TO TUNE THESE VALUES!!!
    LIFT_SPD[0] = 145;
    LIFT_SPD[1] = 145;
  }
  else {
    LIFT_SPD[0] = 135;
    LIFT_SPD[1] = 100;
  }
  lifts_target(target, target);
}

void lifts(String state0, String state1) {
  int target0 = (state0.equals("UP")) ? -10000 : ((state0.equals("DOWN")) ? 10000 : 0);
  int target1 = (state1.equals("UP")) ? -10000 : ((state1.equals("DOWN")) ? 10000 : 0);
  lifts_target(target0, target1);
}

void lifts_target(int target0, int target1) {

  all_lifts[0].target = target0;
  all_lifts[1].target = target1;

  for (int i=0; i<2; i++) {
    int pos    = all_lifts[i].pos;
    int target = all_lifts[i].target;
    all_lifts[i].reached=0;

    if      (pos == target) all_lifts[i].reached=1;
    else if (pos < target)  all_lifts[i].setSpeed(LIFT_SPD[i]);  // positive speed means go DOWN
    else if (pos > target)  all_lifts[i].setSpeed(-LIFT_SPD[i]); // negative speed means go UP
  }

  while(!all_lifts[0].reached || !all_lifts[1].reached) {
    Serial.print(all_lifts[0].target); Serial.print(" "); Serial.print(all_lifts[0].pos);
    Serial.print("\t\t");
    Serial.print(all_lifts[1].target); Serial.print(" "); Serial.println(all_lifts[1].pos);
    delay(10);
  }
}

template <int l>
void readLift() {
  int b = digitalRead(all_lifts[l].encb);

  if (b<=0) all_lifts[l].pos--;
  else      all_lifts[l].pos++;

  // reached target
  if (all_lifts[l].pos == all_lifts[l].target) {
    all_lifts[l].reached = 1;
    all_lifts[l].setSpeed(0);
  }
}

void IMU_setup() {
  Wire.begin();
  Wire.setClock(400000);
  while (true)
  {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying IMU again..."));
      delay(500);
    }
    else break;
  }
  
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok); // Initialize the DMP
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok); // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // // Configuring DMP to output data; set to the maximum
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok); // Enable the FIFO
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok); // Enable the DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok); // Reset DMP
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok); // Reset FIFO
  if (success)
    Serial.println(F("DMP enabled!"));
  else
    Serial.println(F("Enable DMP failed!"));
}

double get_yaw() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {// Was valid data available?
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {// We have asked for GRV data so we should receive Quat6
      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      return yaw;
    }
  }
}
// Align robot to be (roughly) parallel to stairs
void align() {
  double thr=5, yaw;
  while(true) {
    yaw = get_yaw();
    if      (yaw >  thr) move("CW",  MOTOR_SPD, MOTOR_DEL);
    else if (yaw < -thr) move("CCW", MOTOR_SPD, MOTOR_DEL);
    else               { move("OFF", 0); return; }
  }
}

// First half of robot goes up/down
void traverse_0() {
  if (POSITION.equals("BOTTOM")) {
    align();
    while (!front_US_within(5)) // looking at next step
      move("FORWARD", MOTOR_SPD, MOTOR_DEL);
    
    align();
    lifts("DOWN"); // push up
    while (!front_US_within(10)) // looking at future step
      move("FORWARD", MOTOR_SPD, MOTOR_DEL);
  }
  else if (POSITION.equals("TOP")) {
    align();
    while (limitState[0]==0 || limitState[1]==0) // looking down at next step
      move("BACKWARD", MOTOR_SPD, MOTOR_DEL);

    align();
    lifts("DOWN"); // set back lift on next step
  }
}

// Second half of robot goes up/down
void traverse_1() {
  if (POSITION.equals("BOTTOM")) {
    align();
    lifts("UP");
    while (!front_US_within(5)) // looking at next step
      move("FORWARD", MOTOR_SPD, MOTOR_DEL);
  }
  else if (POSITION.equals("TOP")) {
    align();
    move("BACKWARD", MOTOR_SPD, MOTOR_DEL); // needs to be precise enough for the front wheels to get off the previous step
    lifts("DOWN");
  }
}

void escape_operation() {
  if      (POSITION.equals("BOTTOM")) move("FORWARD",  140, 1000);
  else if (POSITION.equals("TOP"))    move("BACKWARD", 140, 1000);

  buzz(); buzz();
}


/** MAIN OPERATION CODE + TEST CASES */
void code() {
//   main_operation();

//  lifts("UP", "UP");     // bring BOTH lifts all the way up
//  lifts("UP", "OFF");    // bring FRONT lift all the way up
//  lifts("OFF", "UP");    // bring BACK lift all the way up
//  lifts("DOWN", "DOWN"); // bring BOTH lifts all the way down
//  lifts("DOWN", "OFF");  // bring FRONT lift all the way down
//  lifts("OFF", "DOWN");  // bring BACK lift all the way down
//  lift_test();

//  move("FORWARD", MOTOR_SPD);
//  left_right_test();
  complex_test();
  
}


void main_operation() {
  buzz(); buzz(); delay(100); // operation is starting!

  vacuum("OFF");
  lifts("UP");
  
  if (front_US_within(10)) POSITION = "BOTTOM";
  else                     POSITION = "TOP";


  for (int i=0; i<STEPS; i++) {
    traverse_0();
    sweep("LEFT");

    traverse_1();
    sweep("RIGHT");
  }
  
  escape_operation();
  
}


void align_test() {
  while(true) {
    move("CW", MOTOR_SPD, 1000);
    align(); delay(3000);
    
    move("CCW", MOTOR_SPD, 1000);
    align(); delay(3000);
  }
}

void vacuum_test() {
  while(true) {
    vacuum("ON");  delay(3000);
    vacuum("OFF"); delay(3000);
  }
}

void left_right_test() {
  while(true) {
    move("LEFT"); delay(5000);
    move("OFF"); delay(2000);
    move("RIGHT"); delay(5000);
    move("OFF"); delay(2000);
  }
}

void complex_test() {
  while(true) {
    move("LEFT"); delay(300);
    move("OFF"); delay(100);
//    move("FORWARD", 50); delay(100);
    motors(120, 80, 120, 80); delay(100);
    move("OFF"); delay(100);
  }
}

void sweep_test() {
  while(true) {
    sweep("LEFT");  delay(2000);
    sweep("RIGHT"); delay(2000);
  }
}

void lift_test() {
  while(true) {
    lifts("DOWN"); delay(2000);
    lifts("UP");   delay(2000);
  }
}

void traverse_upstairs_test() {
  POSITION = "BOTTOM";
  while(true) {
    traverse_0();
    traverse_1();
  }
}
void traverse_downstairs_test() {
  POSITION = "TOP";
  while(true) {
    traverse_0();
    traverse_1();
  }
}
