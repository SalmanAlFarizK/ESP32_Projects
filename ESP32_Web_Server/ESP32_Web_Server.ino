#include <WiFi.h>
#include <WebServer.h>   // Built-in Arduino library

#define USER_LED  (2)

const char* ssid = "Sfk112~";   // Default SSID in Wokwi
const char* password = "12345687";          // No password for Wokwi-GUEST

WebServer server(80);  // Port 80 = HTTP

void handleRoot() {
  String message = "<!DOCTYPE html><html>";
  message += "<head><meta http-equiv='refresh' content='2'></head>"; // refresh every 2 sec
  message += "<body>";
  message += "Hello from ESP32!<br>";
  message += "Millis: " + String(millis());
  message += "</body></html>";
  server.send(200, "text/html", message);
}


void setup() {
  Serial.begin(115200);
  pinMode(USER_LED, OUTPUT);

  WiFi.begin(ssid, password);
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n Connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);   // When browser goes to ESP32’s IP
  server.begin();               // Start web server
  delay(2000);
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
  delay(1000);

  server.handleClient();  // Handle incoming requests
  handleRoot();
}

