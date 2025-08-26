#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define LED_BUILTIN 2

// Settings
static const uint8_t buf_len = 255;     // Size of buffer to look for command
static const char command[] = "delay "; // Note the space!
static const int delay_queue_len = 5;   // Size of delay_queue
static const int msg_queue_len = 5;     // Size of msg_queue
static const uint8_t blink_max = 100;   // Num times to blink before message

// Pins (change this if your Arduino board does not have LED_BUILTIN defined)
static const int led_pin = LED_BUILTIN;

// Message struct for queue communication
typedef struct Message {
  char body[20];
  int count;
} Message;

// Globals
static QueueHandle_t delay_queue;
static QueueHandle_t msg_queue;

//*****************************************************************************
// Task: Command Line Interface
void doCLI(void *parameters) {

  Message rcv_msg;
  char c;
  char buf[buf_len];
  uint8_t idx = 0;
  uint8_t cmd_len = strlen(command);
  int led_delay;

  memset(buf, 0, buf_len);

  while (1) {

    // Check if message available from Blink task
    if (xQueueReceive(msg_queue, (void *)&rcv_msg, 0) == pdTRUE) {
      Serial.print(rcv_msg.body);
      Serial.println(rcv_msg.count);
    }

    // Read from Serial
    if (Serial.available() > 0) {
      c = Serial.read();

      // Store received character in buffer
      if (idx < buf_len - 1) {
        buf[idx] = c;
        idx++;
      }

      // On newline: process command
      if ((c == '\n') || (c == '\r')) {
        buf[idx] = '\0';  // Null terminate buffer
        Serial.print("\r\n");

        // Check if buffer starts with "delay "
        if (memcmp(buf, command, cmd_len) == 0) {
          char *tail = &buf[cmd_len];     // pointer to number after "delay "
          led_delay = atoi(tail);
          led_delay = abs(led_delay);

          // Send delay to blink task
          if (xQueueSend(delay_queue, (void *)&led_delay, 10) != pdTRUE) {
            Serial.println("ERROR: Could not put item on delay queue.");
          }
        }

        // Reset buffer
        memset(buf, 0, buf_len);
        idx = 0;
      } else {
        // Echo characters
        Serial.print(c);
      }
    }
  }
}

//*****************************************************************************
// Task: Blink LED with delay from queue
void blinkLED(void *parameters) {

  Message msg;
  int led_delay = 500;   // Default delay
  uint8_t counter = 0;

  pinMode(led_pin, OUTPUT);

  while (1) {

    // See if new delay value arrived
    if (xQueueReceive(delay_queue, (void *)&led_delay, 0) == pdTRUE) {
      strcpy(msg.body, "Delay updated to ");
      msg.count = led_delay;
      xQueueSend(msg_queue, (void *)&msg, 10);
    }

    // Blink LED
    digitalWrite(led_pin, HIGH);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);

    // Increment blink counter
    counter++;
    if (counter >= blink_max) {
      strcpy(msg.body, "Blinked ");
      msg.count = counter;
      xQueueSend(msg_queue, (void *)&msg, 10);
      counter = 0;
    }
  }
}

//*****************************************************************************
// Main (setup + loop)

void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  Serial.println();
  Serial.println("---FreeRTOS Queue Solution (Fixed)---");
  Serial.println("Enter the command 'delay xxx' where xxx is your desired ");
  Serial.println("LED blink delay time in milliseconds");

  // Create queues
  delay_queue = xQueueCreate(delay_queue_len, sizeof(int));
  msg_queue = xQueueCreate(msg_queue_len, sizeof(Message));

  // Start CLI task
  xTaskCreatePinnedToCore(doCLI,
                          "CLI",
                          2048,
                          NULL,
                          1,
                          NULL,
                          app_cpu);

  // Start Blink task
  xTaskCreatePinnedToCore(blinkLED,
                          "Blink LED",
                          2048,
                          NULL,
                          1,
                          NULL,
                          app_cpu);

  // Delete setup task
  vTaskDelete(NULL);
}

void loop() {
  // Should never get here
}
