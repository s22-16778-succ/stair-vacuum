int IR_PIN = 2;

void setup()
{
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
}

float get_Sharp_GP2Y0A21YK_Distance(int PinID)
{
    // Read analog to digital converter value
    float ADCValue = (float)analogRead(PinID);

    // Convert in millimeters and return distance
    return  200.3775040589502
            - 2.2657665648980 * ADCValue
            + 0.0116395328796 * ADCValue * ADCValue
            - 0.0000299194195 * ADCValue * ADCValue * ADCValue
            + 0.0000000374087 * ADCValue * ADCValue * ADCValue * ADCValue
            - 0.0000000000181 * ADCValue * ADCValue * ADCValue * ADCValue * ADCValue;
}

void loop()
{
  Serial.print("\nDistance in cm: ");
  Serial.print(analogRead(IR_PIN));  
  delay(500); //make it readable
}
