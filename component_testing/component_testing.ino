#define LIMIT_SWITCH_PIN 7
 
void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
}
 
void loop() {
 
  if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) Serial.println("Activated!");
  else                                       Serial.println("Not activated.");
   
  delay(100);
}
