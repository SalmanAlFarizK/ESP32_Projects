#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  
  const char pin[] = "4321";
  // Start Bluetooth with a custom PIN
  SerialBT.begin("ESP32_HelloWorld");  // Bluetooth name
  SerialBT.setPin(pin, 4);             // Set PIN to 4321

  Serial.println("Bluetooth started, now you can pair it using PIN 4321!");
}

void loop() {
  SerialBT.println("Hello World");
  delay(2000);
}
