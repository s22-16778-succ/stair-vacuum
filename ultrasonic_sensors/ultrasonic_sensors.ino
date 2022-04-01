// each ultrasonic sensor is defined by [TRIG, ECHO]
// FRONT_1, FRONT_2, LEFT, RIGHT
int ALL_US[4][2] = {{11,13}, {7,12}, {8,10}, {8,9}};
int LED_5V  = 3;
int LED_GND = 4;

void setup() {
  Serial.begin(9600);
  for (int i=0; i<4; i++) {
    pinMode(ALL_US[i][0], OUTPUT);
    pinMode(ALL_US[i][1], INPUT);
  }

  pinMode(LED_5V, OUTPUT);
  pinMode(LED_GND, OUTPUT);
  digitalWrite(LED_GND, LOW);
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
//  float left_d    = get_distance(ALL_US[2]);
//  float right_d   = get_distance(ALL_US[3]);

  Serial.print("Front 1 (in.): "); Serial.println(front_1_d);
  Serial.print("Front 2 (in.): "); Serial.println(front_2_d);
//  Serial.print("Left (in.): ");    Serial.println(left_d);
//  Serial.print("Right (in.): ");   Serial.println(right_d);

  float p_dif = abs(front_1_d - front_2_d) / 2 / (front_1_d + front_2_d) * 100;
  Serial.print("Percent Difference: "); Serial.print(p_dif); Serial.println("%");
  if( (p_dif < 5 && front_1_d < 5) || (p_dif < 3 && front_1_d < 10) || (p_dif < 1 && front_1_d < 50) )
    analogWrite(LED_5V, 2 * 255.0/5);
  else
    digitalWrite(LED_5V, LOW);
  Serial.println();
  delay(200);
  
}
