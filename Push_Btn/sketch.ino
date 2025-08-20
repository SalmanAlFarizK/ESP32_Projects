#define LED_PIN 18
#define BUTTON_PIN 21

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
  Serial.begin(115200);
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {  // Button pressed
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Button Pressed → LED ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("Button Released → LED OFF");
  }
  delay(100);
}
