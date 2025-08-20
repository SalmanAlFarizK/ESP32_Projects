#include <WiFi.h>

#define USER_LED (18)

const char* ssid = "Wokwi-GUEST";   // Default SSID in Wokwi
const char* password = "";          // No password for Wokwi-GUEST

void setup() {
  Serial.begin(115200);
  pinMode(USER_LED, OUTPUT);
  Serial.print("Connecting to WiFi");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // You can ping connection status
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Still connected to WiFi");
    digitalWrite(USER_LED, HIGH);
  } else {
    Serial.println("Disconnected");
    digitalWrite(USER_LED, LOW);
  }
  delay(2000);
}
