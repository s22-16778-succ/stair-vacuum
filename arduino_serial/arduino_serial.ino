#include "arduino_serial.h"

void setup() {
  Serial.begin(115200);
  for (int i=0; i<4; i++) {
    pinMode(ALL_US[i][0], OUTPUT);
    pinMode(ALL_US[i][1], INPUT);
  }

  pinMode(ALL_BUMPERS[0], INPUT_PULLUP);
  pinMode(ALL_BUMPERS[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ALL_BUMPERS[0]), bumper_left, FALLING);
  attachInterrupt(digitalPinToInterrupt(ALL_BUMPERS[1]), bumper_right, FALLING);

  pinMode(BUZZER, OUTPUT);

  for (int i=0; i<4; i++) {
    pinMode(MOTORS[i][0], OUTPUT);
    pinMode(MOTORS[i][1], OUTPUT);
    pinMode(MOTORS[i][2], OUTPUT);
  }

  pinMode(VACUUM[0], OUTPUT);
  pinMode(VACUUM[1], OUTPUT);

  // Lifts
  for (int i=0; i<2; i++) {
    pinMode(LIFTS[i][0], OUTPUT);
    pinMode(LIFTS[i][1], OUTPUT);
    pinMode(LIFTS[i][2], OUTPUT);
    pinMode(LIFTS[i][3], INPUT);
    pinMode(LIFTS[i][4], INPUT);
    LIFTS[i][5] = 0;
  }
  attachInterrupt(digitalPinToInterrupt(LIFTS[0][3]), readLift0, RISING);
  attachInterrupt(digitalPinToInterrupt(LIFTS[1][3]), readLift1, RISING);

  // IMU setup
  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying IMU again..."));
      delay(500);
    }
    else initialized = true;
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


void loop() {
  if (Serial.available() > 0) {
    // Read in Raspberry Pi command
    Serial.readStringUntil('\n').toCharArray(command, 100);
    ptr = strtok(command, ",");
    int index = 0;
    while (ptr != NULL) {
      strings[index++] = ptr;
      ptr = strtok(NULL, ",");
    }

    // Interpret the strings
    // for (int i=0; i<index; i++)
    //  Serial.println(strings[i]);
    
    if (strings[0].equals("get_US"))
      get_US(strings[1]);
    if (strings[0].equals("buzz"))
      buzz();

  }
//   Serial.println(get_distance(ALL_US[0]));
}

double get_distance(int U[]) {
  // Trigger
  digitalWrite(U[0], LOW);
  delayMicroseconds(5);
  digitalWrite(U[0], HIGH);
  delayMicroseconds(10);
  digitalWrite(U[0], LOW);

  // Echo
  double duration = pulseIn(U[1], HIGH);
  delay(50);
  return min(duration * SOUND_SPEED / 2, 100); // inches
}

double get_US(String sensor) {
  if      (sensor.equals("FRONT_LEFT"))  return get_distance(ALL_US[0]);
  else if (sensor.equals("FRONT_RIGHT")) return get_distance(ALL_US[1]);
  else if (sensor.equals("LEFT"))        return get_distance(ALL_US[2]);
  else if (sensor.equals("RIGHT"))       return get_distance(ALL_US[3]);
}

void bumper_left() {
  if (millis() - lastDebounce < DEBOUNCE_DELAY) return;
  lastDebounce = millis();
  bumperState = 1;
}
void bumper_right() {
  if (millis() - lastDebounce < DEBOUNCE_DELAY) return;
  lastDebounce = millis();
  bumperState = 2;
}
void buzz() {
  tone(BUZZER, 1000);
  delay(100);
  noTone(BUZZER);
}

// Move/Rotate robot for delay seconds (speed is positive)
void move(String dir, double spd, double del) {
  if      (dir.equals("FORWARD"))  motors( spd,  spd,  spd,  spd);
  else if (dir.equals("BACKWARD")) motors(-spd, -spd, -spd, -spd);
  else if (dir.equals("LEFT"))     motors(-spd,  spd,  spd, -spd);
  else if (dir.equals("RIGHT"))    motors( spd, -spd, -spd,  spd);
  else if (dir.equals("CW"))       motors( spd, -spd,  spd, -spd);
  else if (dir.equals("CCW"))      motors(-spd,  spd, -spd,  spd);

  delay(del);
  motors(0,0,0,0);
}
// Align robot to be (roughly) parallel to stairs
void align() {
  double spd=80, del=10, thr=5, yaw=0;
  while(true) {
    yaw = get_yaw();
    if      (yaw > thr) move("CW",  spd, del);
    else if (yaw <-thr) move("CCW", spd, del);
    else {
      motors(0,0,0,0);
      return;
    }
  }
}

// Run the front left motor, front right motor, back left motor, and back right motor
// spd must be from [-255, 255]
void motors(double spd0, double spd1, double spd2, double spd3) {
  motor(0, spd0);
  motor(1, spd1);
  motor(2, spd2);
  motor(3, spd3);
}
void motor(int i, double spd) {
  if (spd == 0) {
    digitalWrite(MOTORS[i][0], LOW);
    digitalWrite(MOTORS[i][1], LOW);
  }
  else if (spd > 0) {
    analogWrite(MOTORS[i][2], spd);
    digitalWrite(MOTORS[i][0], HIGH);
    digitalWrite(MOTORS[i][1], LOW);
  }
  else if (spd < 0) {
    analogWrite(MOTORS[i][2], -spd);
    digitalWrite(MOTORS[i][0], LOW);
    digitalWrite(MOTORS[i][1], HIGH);
  } 
}

// Turn vacuum ON or OFF
void vacuum(String state) {
  if (state.equals("ON")) {
    digitalWrite(VACUUM[0], HIGH);
    digitalWrite(VACUUM[0], LOW);
  }
  else if (state.equals("OFF")) {
    digitalWrite(VACUUM[0], LOW);
    digitalWrite(VACUUM[0], LOW);
  }
}
void sweep(String state) {
  double spd=80, del=10, thr=5;
  vacuum("ON");
  
  while(True) {
    if (state.equals("LEFT")) {
      motors("LEFT", spd, del);
      if (bumperState == 1) break;
    }
    else if (state.equals("RIGHT")) {
      motors("RIGHT", spd, del);
      if (bumperState == 2) break;
    }
  }
  
  bumperState = 0;
  vacuum("OFF");
  motors(0,0,0,0);
}

void lifts(String state) {
  if (state.equals("UP")) {
    // ???
    setMotor(1, 0, LIFT[0][0], LIFT[0][1], LIFT[0][2]);
    setMotor(1, 0, LIFT[1][0], LIFT[1][1], LIFT[1][2]);
  }
  else if (state.equals("DOWN")) {
    // ???
    setMotor(-1, 0, LIFT[0][0], LIFT[0][1], LIFT[0][2]);
    setMotor(-1, 0, LIFT[1][0], LIFT[1][1], LIFT[1][2]);
  }
}
void setMotor(int dir, int pwmVal, int in1, int in2, int pwm){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readLift(int i) {
  int ENCB = digitalRead(LIFTS[i][3]);
  if (ENCB>0) LIFTS[i][4]++;
  else        LIFTS[i][4]--;
}
void readLift0() { readLift(0); }
void readLift1() { readLift(1); }

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

void escape_operation() {
  if      (POSITION.equals("BOTTOM")) move("FORWARD",  140, 1000);
  else if (POSITION.equals("TOP"))    move("BACKWARD", 140, 1000);

  buzz(); buzz();
}


// First half of robot goes up/down
void traverse_0() {
  double spd=80, del=10;
  if (POSITION.equals("BOTTOM")) {
    align();
    while (get_US("FRONT_LEFT")>5 && get_US("FRONT_RIGHT")>5) // looking at next step
      move("FORWARD", spd, del);
    
    align();
    lifts("DOWN"); // push up
    while (get_US("FRONT_LEFT")>10 && get_US("FRONT_RIGHT")>10) // looking at future step
      move("FORWARD", spd, del);
  }
  else if (POSITION.equals("TOP")) {
    align();
    while (not_on_next_step) // looking down at next step
      move("BACKWARD", spd, del);

    align();
    lifts("DOWN");
  }
}

// Second half of robot goes up/down
void traverse_1() {
  double spd=80, del=10;
  if (POSITION.equals("BOTTOM")) {
    align();
    lifts("UP");
    while (get_US("FRONT_LEFT")>5 && get_US("FRONT_RIGHT")>5) // looking at next step
      move("FORWARD", spd, del);
  }
  else if (POSITION.equals("TOP")) {
    align();
    move("BACKWARD", spd, del) // needs to be precise enough for the front wheels to get off the previous step
    lifts("DOWN");
  }
}

void main_operation() {
  buzz(); delay(100); // warn that operation is starting!

  vacuum("OFF");
  lifts("UP");
  
  if (get_US("FRONT_LEFT")<10 && get_US("FRONT_RIGHT")<10)
    POSITION = "BOTTOM";
  else
    POSITION = "TOP";

  while(STEPS != PASSES) {
    traverse_0();
    sweep("LEFT");

    traverse_1();
    sweep("RIGHT");
    PASSES++;
  }

  escape_operation();
}
