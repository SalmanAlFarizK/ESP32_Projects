#include <SPI.h>
#include <SD.h>
#include <string.h>
 
 
 
/*************************************
* Pin map.
*************************************/
#define SD_CS 5
#define PIN_RED    25
#define PIN_GREEN  26
#define PIN_BLUE   27
 
#define TRIGGER_PIN 4
#define ECHO_PIN    17
 
// ----------------- Constants ----------------
#define SOUND_SPEED       0.034f    // cm/us
#define CM_TO_INCH        0.393701f
#define MAX_NEAREST_DISTANCE_THRESHOLD_CM  10
 
// ----------------- Globals ------------------
long duration_us;
float distance_cm;
float distance_in;
unsigned long currentMillis;
unsigned long previousMillis = 0;
 
typedef struct {
  uint32_t uiFlahSaveSize;
  uint32_t uiObcTime;              // simple seconds counter
} DistanceCrossInfo;
 
DistanceCrossInfo gtDistancInfo;
 
/******************** Ultrasonic ********************/
int UltraSonicSensorReading(void) {
  // Trigger pulse
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
 
  // Echo read
  duration_us = pulseIn(ECHO_PIN, HIGH, 30000UL); // timeout 30ms
  if (duration_us == 0) {
    return 9999; // out of range / timeout
  }
 
  distance_cm = duration_us * SOUND_SPEED / 2.0f;
  distance_in = distance_cm * CM_TO_INCH;
  return (int)distance_cm;
}
 
/******************** RGB LED ********************/
void setColor(int R, int G, int B) {
  analogWrite(PIN_RED,   R);
  analogWrite(PIN_GREEN, G);
  analogWrite(PIN_BLUE,  B);
}
 
void RgbLedInit(void) {
  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);
  // start off
  setColor(0, 0, 0);
}
 
 
/*************************************
* SD Card Functions.
*************************************/
void InitSdCard(void)
{
 
  Serial.println("Initializing SD card...");
 
  if (!SD.begin(SD_CS))
  {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("SD Card initialized.");
 
  return;
}
void WriteToSdCard(const char* pucDataMsg)
{
  File dataFile = SD.open("/Sample_.txt", FILE_WRITE);  // use same file always
 
  if (dataFile) {
    dataFile.println(pucDataMsg);
    dataFile.close();
    Serial.println("Wrote to Sample_.txt");
  } else {
    Serial.println("Error opening Sample_.txt for writing");
  }
}
 
void ReadFromSdCard(void)
{
  File dataFile = SD.open("/Sample_.txt", FILE_READ);  // same file
 
  if (dataFile) {
    Serial.println("Reading Sample_.txt:");
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
    Serial.println();
  } else {
    Serial.println("Error opening Sample_.txt for reading");
  }
}
 
 
/******************** Distance Alert FSM ********************/
void DistanceAlertFsm(void) {
  int dist = UltraSonicSensorReading();
 
  // Build message buffer
  uint8_t msg[150] = {0};
 
  if (dist < MAX_NEAREST_DISTANCE_THRESHOLD_CM) {
    setColor(255, 0, 0);  // RED
 
    int msgLen = snprintf((char*)msg, sizeof(msg),
                          "The Distance is %d cm, OBC time: %u s",
                          dist, gtDistancInfo.uiObcTime);
 
    // (Optional) Save to flash
    WriteToSdCard((char*)msg);
 
    Serial.print("ALERT -> ");
    Serial.println((char*)msg);
  }
  else {
    setColor(0, 255, 0);  // GREEN
    // You may also send the normal distance periodically if you want:
    // snprintf((char*)msg, sizeof(msg), "Distance OK: %d cm", dist);
    // Blynk.virtualWrite(V1, (char*)msg);
  }
}
 
/******************** Time Ticker (1s) ********************/
void TimeTicker(void) {
  currentMillis = millis();
  if ((currentMillis - previousMillis) >= 1000UL) {
    previousMillis = currentMillis;
    gtDistancInfo.uiObcTime += 1;
  }
}
 
/******************** Setup & Loop ********************/
void setup() {
  Serial.begin(115200);
  Serial.println("Setup function Entered.");
 
 
  RgbLedInit();
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  InitSdCard();
  ReadFromSdCard();
 
 
  Serial.println("Setup done.");
}
 
void loop() {
 
  TimeTicker();
  DistanceAlertFsm();
 
  // Keep delays small so Blynk stays responsive
  delay(100);
}
 