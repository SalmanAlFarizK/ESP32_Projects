/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-rgb-led
 */
#include <SPI.h>
#include <string.h>

#define PIN_RED           25 // GPIO23
#define PIN_GREEN         26 // GPIO22
#define PIN_BLUE          27 // GPIO21
#define TRIGGER_PIN       4
#define ECHO_PIN          17
#define SOUND_SPEED       0.034
#define CM_TO_INCH        0.393701
#define PIN_CS            5


#define MAX_NEAREST_DISTANCE_THRESHOLD_CM    10
#define MAX_READ_SIZE 60

long duration;
float distanceCm;
float distanceInch;

typedef struct _DistanceCrossInfo_
{
  uint32_t uiFlahSaveSize;
  uint32_t uiObcTime;
} DistanceCrossInfo;

DistanceCrossInfo gtDistancInfo;
unsigned long currentMillis;
unsigned long previousMillis = 0;   // stores last tick

/*******************************************************************
* W25QxxFlash functions.
*******************************************************************/
uint32_t readJEDECID() 
{
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x9F);  // JEDEC ID command

  uint8_t mfg = SPI.transfer(0x00);
  uint8_t mem = SPI.transfer(0x00);
  uint8_t cap = SPI.transfer(0x00);

  digitalWrite(PIN_CS, HIGH);

  return (mfg << 16) | (mem << 8) | cap;
}

void writeEnable() 
{
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x06);  // Write Enable command
  digitalWrite(PIN_CS, HIGH);
}

void flashWaitBusy() {
  uint8_t status;
  do {
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(0x05);       // RDSR
    status = SPI.transfer(0); // read
    digitalWrite(PIN_CS, HIGH);
  } while (status & 0x01); // Wait for BUSY=0
}

void flashPageProgram(uint32_t addr, uint8_t *data, uint16_t len) {
  // Step 1: Erase the sector that contains this address
  uint32_t sectorAddr = addr & ~(0xFFF); // align to 4KB boundary
  writeEnable();
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x20);  // Sector Erase (4KB)
  SPI.transfer((sectorAddr >> 16) & 0xFF);
  SPI.transfer((sectorAddr >> 8) & 0xFF);
  SPI.transfer(sectorAddr & 0xFF);
  digitalWrite(PIN_CS, HIGH);
  flashWaitBusy();

  // Step 2: Write up to 100 bytes into the page
  writeEnable();
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x02);  // Page Program
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);

  for (uint16_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }

  digitalWrite(PIN_CS, HIGH);
  flashWaitBusy();
}

void flashReadData(uint32_t addr, uint8_t *buffer, uint16_t len) 
{
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x03);  // Read Data
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);

  for (int i = 0; i < len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_CS, HIGH);
}



void chipErase() 
{
  writeEnable();
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0xC7);  // Chip Erase command
  digitalWrite(PIN_CS, HIGH);

  Serial.println("Erasing... this may take some time");
  delay(5000); // wait for erase to finish
}

void SaveToFlash(uint8_t* ucData, uint32_t uiDataSize)
{
  flashPageProgram(0, ucData, uiDataSize);
}

void ReadFromFlash(uint8_t* ucOutputBuff, uint32_t uiReadSize)
{
  flashReadData(0, ucOutputBuff, uiReadSize);
}

void ReadOnBoot(void)
{
  uint8_t ucReadBuff[100] = {0};

  ReadFromFlash(ucReadBuff, MAX_READ_SIZE);
  delay(1000);

  Serial.println("The data read from flash on boot:");
  for(int i = 0; i < MAX_READ_SIZE; ++i)
  {
    Serial.print((char)ucReadBuff[i]);
  }
  Serial.println();
  delay(100);
}


/*******************************************************************
* Ultra sonic sensor functions.
*******************************************************************/
int UltraSonicSensorReading(void)
{
  // Clears the trigPin
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  
  // Prints the distance in the Serial Monitor
  // Serial.print("Distance (cm): ");
  // Serial.println(distanceCm);
  // delay(1000);

  return (int)distanceCm;
}

/*******************************************************************
* RGB LED functions.
*******************************************************************/
void RgbLed(void)
{
  // Blink RED only
  setColor(255, 0, 0);  // RED on
  delay(1000);
  setColor(0, 0, 0);    // All off
  delay(1000);

  // Blink GREEN only
  setColor(0, 255, 0);  // GREEN on
  delay(1000);
  setColor(0, 0, 0);    // All off
  delay(1000);

  // Blink BLUE only
  setColor(0, 0, 255);  // BLUE on
  delay(1000);
  setColor(0, 0, 0);    // All off
  delay(1000);

  return;
}

void setColor(int R, int G, int B) 
{
  analogWrite(PIN_RED,   R);
  analogWrite(PIN_GREEN, G);
  analogWrite(PIN_BLUE,  B);
}

/*******************************************************************
* Distance Alert FSM
*******************************************************************/
void DistanceAlertFsm(void)
{
  int iMaxNearestDistance = 0;
  uint8_t ucFlahSaveBuff[150] = {0};
  int uiFlshSaveMsgSize = 0;

  iMaxNearestDistance = UltraSonicSensorReading();

  if(iMaxNearestDistance < MAX_NEAREST_DISTANCE_THRESHOLD_CM)
  {
    setColor(255, 0, 0);  // RED on
    uiFlshSaveMsgSize = sprintf((char*)ucFlahSaveBuff, "The Distance is %d And the OBC time is : %d",
                        iMaxNearestDistance,gtDistancInfo.uiObcTime);

    SaveToFlash(ucFlahSaveBuff, uiFlshSaveMsgSize);
    Serial.println("Saved new distance and time to flash");
  }
  else
  {
    setColor(0, 255, 0);  // GREEN on
    //Serial.print("The distance is :");
    //Serial.println(iMaxNearestDistance);
  }
}

/*******************************************************************
* Initialization functions.
*******************************************************************/
void RgbLedInit(void)
{
  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);
  return;
}

void UltraSonicSensorInit(void)
{
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  return;
}

void W25QxxFlashInit(void)
{  
  SPI.begin(18, 19, 23, PIN_CS);  // SCK=18, MISO=19, MOSI=23, CS=5
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  return;
}
/*******************************************************************
* Time Ticker
*******************************************************************/
void TimeTicker(void)
{ 
  currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) // 1 second passed
  {
    gtDistancInfo.uiObcTime += 1;  // increment your time counter
    previousMillis = currentMillis; // update last tick
  }
}



/*******************************************************************
* Start of Setup and Loop
*******************************************************************/
void setup()
{
  RgbLedInit();
  UltraSonicSensorInit();
  W25QxxFlashInit();
  Serial.begin(115200); // Starts the serial communication]

  ReadOnBoot();
}

void loop()
{
  TimeTicker();
  DistanceAlertFsm();

  delay(1000);
}