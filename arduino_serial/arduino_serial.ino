String strings[10];
char command[100];
char *ptr = NULL;

// [TRIG, ECHO] for FRONT_LEFT, FRONT_RIGHT, LEFT, RIGHT
int ALL_US[4][2] = {{52,50}, {53,51}, {49,47}, {48,46}};
float SOUND_SPEED = 0.0135039; // inches per microsecond

// Bumpers
int ALL_BUMPERS[2] = {2, 3};
int DEBOUNCE_DELAY = 50; // ms
int lastDebounce = 0;

// Buzzer
int BUZZER = 4;

void setup() {
  Serial.begin(9600);
  for (int i=0; i<4; i++) {
    pinMode(ALL_US[i][0], OUTPUT);
    pinMode(ALL_US[i][1], INPUT);
  }

  pinMode(ALL_BUMPERS[0], INPUT_PULLUP);
  pinMode(ALL_BUMPERS[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ALL_BUMPERS[0]), bumper_left, FALLING);
  attachInterrupt(digitalPinToInterrupt(ALL_BUMPERS[1]), bumper_right, FALLING);

  pinMode(BUZZER, OUTPUT);
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
//    for (int i=0; i<index; i++)
//      Serial.println(strings[i]);
    
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

void get_US(String sensor) {
  if      (sensor.equals("FRONT_LEFT"))  Serial.println(get_distance(ALL_US[0]));
  else if (sensor.equals("FRONT_RIGHT")) Serial.println(get_distance(ALL_US[1]));
  else if (sensor.equals("LEFT"))        Serial.println(get_distance(ALL_US[2]));
  else if (sensor.equals("RIGHT"))       Serial.println(get_distance(ALL_US[3]));
}

void bumper_left() {
  if (millis() - lastDebounce < DEBOUNCE_DELAY) return;
  lastDebounce = millis();
  Serial.println("b_left");
}
void bumper_right() {
  if (millis() - lastDebounce < DEBOUNCE_DELAY) return;
  lastDebounce = millis();
  Serial.println("b_right");
}
void buzz() {
  Serial.println("buzz");
  tone(BUZZER, 1000);
  delay(100);
  noTone(BUZZER);
}
