#define POT_PIN 34

void setup() {
  Serial.begin(115200);
}

void loop()
 {
  float fAnalogVal = 0;
  int value = analogRead(POT_PIN);   // 0 - 4095 (12-bit ADC on ESP32)

  fAnalogVal = (float)(value*3.3) / 4095;
  Serial.println(fAnalogVal);
  delay(500);
}
