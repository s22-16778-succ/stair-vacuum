// each ultrasonic sensor is defined by [TRIG, ECHO]
// FRONT_1, FRONT_2, LEFT, RIGHT
int ALL_US[4][2] = {{52,50}, {53,51}, {49,47}, {48,46}};
float SOUND_SPEED = 0.0135039; // inches per microsecond
int old_time;

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
  delayMicroseconds(5);
  digitalWrite(U[0], HIGH);
  delayMicroseconds(10);
  digitalWrite(U[0], LOW);

  // Echo
  double duration = pulseIn(U[1], HIGH);
  return min(duration * SOUND_SPEED / 2, 100); // inches
}

float percent_dif(float x1, float x2) {
  return abs(x1 - x2) / 2 / (x1 + x2) * 100;
//  if( (p_dif < 5 && front_1_d < 5) || (p_dif < 3 && front_1_d < 10) || (p_dif < 1 && front_1_d < 50) )
//    analogWrite(LED_5V, 2 * 255.0/5);
//  else
//    digitalWrite(LED_5V, LOW);
}


void loop() {
  old_time = millis();
  
  // need >50ms delay between readings
  float front_1_d = get_distance(ALL_US[0]); delay(50);
  float front_2_d = get_distance(ALL_US[1]); delay(50);
  float left_d    = get_distance(ALL_US[2]); delay(50);
  float right_d   = get_distance(ALL_US[3]); delay(50);

  Serial.print("\t"); Serial.print(front_1_d);
  Serial.print("\t"); Serial.print(front_2_d); Serial.println();
  Serial.print("\t¯¯¯¯¯   ¯¯¯¯¯"); Serial.println();
  Serial.print(left_d); Serial.print("\t|            | "); Serial.print(right_d);
  //  Serial.println(millis() - old_time);
  Serial.print("\n\n\n\n\n\n\n\n\n\n\n");
}
