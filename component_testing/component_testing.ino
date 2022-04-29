int SWITCH = 50;
int LED = 13;

void setup() {
  Serial.begin(115200);
  pinMode(SWITCH, INPUT);
  pinMode(LED, OUTPUT);
}

void loop() {
//  if (digitalRead(SWITCH)) digitalWrite(LED);
//  else(digitalRead(SWITCH)) digitalWrite(LED);
  Serial.println(!digitalRead(SWITCH));
}
