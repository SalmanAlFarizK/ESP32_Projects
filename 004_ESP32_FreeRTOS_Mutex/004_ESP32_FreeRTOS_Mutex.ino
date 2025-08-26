#include <Arduino.h>

// Shared variable
volatile int sharedCounter = 0;

// Mutex handle
SemaphoreHandle_t xMutex;

// Task handles
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;

void Task1(void *pvParameters) {
  for (;;) {
    // Take the mutex before accessing sharedCounter
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      sharedCounter++;
      Serial.printf("Task1 incremented counter to %d\n", sharedCounter);

      // Release the mutex after use
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task2(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      sharedCounter++;
      Serial.printf("Task2 incremented counter to %d\n", sharedCounter);
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(1500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Create the mutex
  xMutex = xSemaphoreCreateMutex();

  if (xMutex != NULL) {
    // Create tasks
    xTaskCreate(Task1, "Task1", 2048, NULL, 1, &Task1Handle);
    xTaskCreate(Task2, "Task2", 2048, NULL, 1, &Task2Handle);
  } else {
    Serial.println("Failed to create mutex");
  }
}

void loop() {
  // Nothing here, tasks handle the work
}
