#include <WiFi.h>
#include <WebServer.h>

#define USER_LED 2   // GPIO2 is onboard LED on many ESP32 boards

const char* ssid = "Sfk112~";     
const char* password = "12345687";  

WebServer server(80);  // Port 80 = HTTP

// ====== HTML page with two buttons ======
String htmlPage() {
  String page = "<!DOCTYPE html><html>";
  page += "<head><title>ESP32 LED Control</title></head>";
  page += "<body style='text-align:center; font-family:Arial;'>";
  page += "<h2>ESP32 LED Control</h2>";
  page += "<p><a href='/on'><button style='padding:20px; font-size:20px;'>LED ON</button></a></p>";
  page += "<p><a href='/off'><button style='padding:20px; font-size:20px;'>LED OFF</button></a></p>";
  page += "</body></html>";
  return page;
}

// ====== Handlers ======
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleLedOn() {
  digitalWrite(USER_LED, HIGH);
  server.send(200, "text/html", htmlPage() + "<p>LED is ON</p>");
}

void handleLedOff() {
  digitalWrite(USER_LED, LOW);
  server.send(200, "text/html", htmlPage() + "<p>LED is OFF</p>");
}

void setup() {
  Serial.begin(115200);
  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, LOW);

  WiFi.begin(ssid, password);
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ Connected to WiFi!");
  Serial.print("📡 ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Set routes
  server.on("/", handleRoot);
  server.on("/on", handleLedOn);
  server.on("/off", handleLedOff);

  server.begin();
}

void loop() {
  server.handleClient();  // Process incoming HTTP requests
}
