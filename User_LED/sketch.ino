#define LED_PIN 2  

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("LED Turned On");
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED Turned Off");
  delay(1000);
}
