/*****************************************************
 * ESP32 + Ultrasonic + RGB LED + W25Qxx + Blynk IoT
 * Cleaned & Corrected
 *****************************************************/

#include <SPI.h>
#include <string.h>
#include <WiFi.h>

// --- Blynk (define BEFORE including Blynk headers)
#define BLYNK_TEMPLATE_ID   "TMPL3XxsUjBpt"
#define BLYNK_TEMPLATE_NAME "ESP32Test"
#define BLYNK_AUTH_TOKEN    "NIXa9FTR5PLXEqLPHXq0Ld4eBy8WMkmG"
#include <BlynkSimpleEsp32.h>

// ----------------- Pin Map -----------------
#define PIN_RED    25
#define PIN_GREEN  26
#define PIN_BLUE   27

#define TRIGGER_PIN 4
#define ECHO_PIN    17

// SPI for external flash (W25Qxx)
#define PIN_CS      5
#define SPI_SCK     18
#define SPI_MISO    19
#define SPI_MOSI    23

// ----------------- Constants ----------------
#define SOUND_SPEED       0.034f    // cm/us
#define CM_TO_INCH        0.393701f
#define MAX_NEAREST_DISTANCE_THRESHOLD_CM  10
#define MAX_READ_SIZE     60

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

// ----------------- WiFi creds ----------------
char ssid[] = "Sfk112~";      // <-- change me
char pass[] = "12345687";  // <-- change me

/******************** WiFi + Blynk ************************/
void Wifi_Init(void) {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.print("\nWiFi Connected. IP: ");
  Serial.println(WiFi.localIP());

  // Start Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("Connected to Blynk.");
}

/******************** W25Qxx Flash helpers ****************/
// Returns 24-bit JEDEC ID (Manufacturer/MemoryType/Capacity)
uint32_t readJEDECID() {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x9F);
  uint8_t mfg = SPI.transfer(0x00);
  uint8_t mem = SPI.transfer(0x00);
  uint8_t cap = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  return ( (uint32_t)mfg << 16 ) | ( (uint32_t)mem << 8 ) | cap;
}

void writeEnable() {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x06);
  digitalWrite(PIN_CS, HIGH);
}

void flashWaitBusy() {
  uint8_t status;
  do {
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(0x05);             // Read Status Reg
    status = SPI.transfer(0x00);
    digitalWrite(PIN_CS, HIGH);
  } while (status & 0x01);          // busy bit
}

void flashPageProgram(uint32_t addr, uint8_t *data, uint16_t len) {
  // Erase 4KB sector containing addr
  uint32_t sectorAddr = addr & ~(0xFFF);
  writeEnable();
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x20); // Sector Erase 4KB
  SPI.transfer((sectorAddr >> 16) & 0xFF);
  SPI.transfer((sectorAddr >> 8) & 0xFF);
  SPI.transfer(sectorAddr & 0xFF);
  digitalWrite(PIN_CS, HIGH);
  flashWaitBusy();

  // Program
  writeEnable();
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x02); // Page Program
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (uint16_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(PIN_CS, HIGH);
  flashWaitBusy();
}

void flashReadData(uint32_t addr, uint8_t *buffer, uint16_t len) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x03); // Read Data
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (uint16_t i = 0; i < len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_CS, HIGH);
}

void SaveToFlash(uint8_t* ucData, uint32_t uiDataSize) {
  flashPageProgram(0, ucData, uiDataSize);
}

void ReadFromFlash(uint8_t* ucOutputBuff, uint32_t uiReadSize) {
  flashReadData(0, ucOutputBuff, uiReadSize);
}

void ReadOnBoot(void) {
  uint8_t ucReadBuff[MAX_READ_SIZE] = {0};
  ReadFromFlash(ucReadBuff, MAX_READ_SIZE);
  delay(100);

  Serial.println("Flash read @ boot:");
  for (int i = 0; i < MAX_READ_SIZE; ++i) {
    if (ucReadBuff[i] == 0) break;
    Serial.print((char)ucReadBuff[i]);
  }
  Serial.println();
}

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

    // Send STRING to Blynk Label on V1
    Blynk.virtualWrite(V1, (char*)msg);

    // (Optional) Save to flash
    // SaveToFlash(msg, msgLen);

    Serial.print("ALERT -> ");
    Serial.println((char*)msg);
  } else {
    setColor(0, 255, 0);  // GREEN
    // You may also send the normal distance periodically if you want:
    // snprintf((char*)msg, sizeof(msg), "Distance OK: %d cm", dist);
    // Blynk.virtualWrite(V1, (char*)msg);
  }
}

/******************** SPI Flash Init ********************/
void W25QxxFlashInit(void) {
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  uint32_t id = readJEDECID();
  Serial.printf("W25Qxx JEDEC ID: 0x%06lX\n", (unsigned long)id);
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
  delay(100);

  RgbLedInit();
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  W25QxxFlashInit();
  Wifi_Init();             // connect WiFi + Blynk
  ReadOnBoot();            // optional

  Serial.println("Setup done.");
}

void loop() {
  // Keep Blynk connection responsive
  Blynk.run();

  TimeTicker();
  DistanceAlertFsm();

  // Keep delays small so Blynk stays responsive
  delay(100);
}
