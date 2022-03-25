// each ultrasonic sensor is defined by [TRIG, ECHO]
//FRONT_1 = [11, 13];
//FRONT_2 = [11, 12];
//LEFT    = [8, 10];
//RIGHT   = [8, 9];
int ALL_US[4][2] = {{11,13}, {11,12}, {8,10}, {8,9}};

void setup() {
  Serial.begin(9600);
  for (int i=0; i<4; i++) {
    pinMode(ALL_US[i][0], OUTPUT);
    pinMode(ALL_US[i][1], INPUT);
  }
}

double get_distance(int U[]) {
  // Trigger
  digitalWrite(U[0], LOW);
  delayMicroseconds(2);
  digitalWrite(U[0], HIGH);
  delayMicroseconds(10);
  digitalWrite(U[0], LOW);

  // Echo
  double duration = pulseIn(U[1], HIGH);
  return duration * 0.0133 / 2; // inches
}

void loop() {
  // put your main code here, to run repeatedly:
  float front_1_d = get_distance(ALL_US[0]);
  float front_2_d = get_distance(ALL_US[1]);
  float left_d    = get_distance(ALL_US[2]);
  float right_d   = get_distance(ALL_US[3]);
  Serial.print("Front 1 (in.): "); Serial.println(front_1_d);
  Serial.print("Front 2 (in.): "); Serial.println(front_2_d);
  Serial.print("Left (in.): ");    Serial.println(left_d);
  Serial.print("Right (in.): ");   Serial.println(right_d);
  Serial.println();
  
}
