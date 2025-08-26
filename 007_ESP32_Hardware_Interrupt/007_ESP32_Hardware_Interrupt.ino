#define LED_BUILTIN 2   // Onboard LED pin (GPIO2)

hw_timer_t *timer = NULL;
volatile bool toggleFlag = false;   // ISR -> main loop communication

// ISR (must be very short)
void IRAM_ATTR onTimer() {
  static bool led_state = false;
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);

  toggleFlag = true;   // set flag for loop()
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Timer Interrupt Example Started");

  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize timer at 1 MHz (tick = 1 µs)
  timer = timerBegin(1000000);

  // Attach ISR
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm: 500000 µs = 0.5s, autoreload
  timerAlarm(timer, 500000, true, 0);
}

void loop() {
  if (toggleFlag) {
    toggleFlag = false;   // clear flag

    static int count = 0;
    Serial.print("Timer interrupt count: ");
    Serial.println(++count);
  }
}
