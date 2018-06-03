
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "esp_task_wdt.h"
#include <math.h>

const char uPhA = 25;   // x encoder
const char uPhB = 26;
const char vPhA = 12;
const char vPhB = 13;
const char wPhA = 14;
const char wPhB = 15;
#define get2bits(startBit, value) ((unsigned char)((value >> startBit) & 3))
const volatile int* GPIOP = (volatile int *)GPIO_IN_REG;

const static int changeTable[4][4] = {
 //00  01  10  11 - prevSwitches
  { 0,  1, -1,  0}, //00 - switches
  {-1,  0,  0,  1}, //01
  { 1,  0,  0, -1}, //10
  { 0, -1,  1,  0}  //11
};

TaskHandle_t decoderTask;

volatile long int uRaw;   // angle between vertical pillar and arm b
volatile long int vRaw;   // angle between arm b and arm c
volatile long int wRaw;   // rotation around vertical pillar

volatile long sampleCount = 0;
long sampleMillis = 0;

typedef struct {
  float x, y, z;
} Point3D;

// measure
const int revSteps = 120000;   // encoder steps per revolution
const int a = 255 + 6.7;  // pillar length [mm]
const int b = 212;  // arm b length [mm]
const int c = 232 - 17.13 / 2;  // arm c length [mm]
const float uZero = 90 * M_PI / 180;
const float vZero = 90 * M_PI / 180;
const float wZero = 0;

void calcXYZ(Point3D &p, long int uRaw, long int vRaw, long int wRaw) {
  // get angles in radians
  float u = (uRaw * M_PI * 2 / revSteps) + uZero;
  float v = (vRaw * M_PI * 2 / revSteps) + vZero;
  float w = (wRaw * M_PI * 2 / revSteps) + wZero;
  // get cartesian coordinates
  float r = b * sin(u) - c * sin(u + v);
  p.x = r * sin(w);
  p.y = -r * cos(w);
  p.z = a - b * cos(u) + c * cos(u + v);
}

void printCoords() {
  Point3D p;
  calcXYZ(p, uRaw, vRaw, wRaw);

  Serial.print(p.x);
  Serial.print(", ");
  Serial.print(p.y);
  Serial.print(", ");
  Serial.println(p.z);
}

void printRawAngles() {
  Serial.print(uRaw);
  Serial.print(", ");
  Serial.print(vRaw);
  Serial.print(", ");
  Serial.println(wRaw);
}

void uiTask(void *parameter) {
  Serial.print("ui core ");
  Serial.println(xPortGetCoreID());

  while (1) {
    if (Serial.available() > 0) {
      char ch = Serial.read();
      if (ch == 'p') {
        printCoords();
      } else if (ch == 'a') {
        printRawAngles();
      } else if (ch == 'c') {
        wRaw = vRaw = uRaw = 0;
      } else if (ch == 's') {
        long now = millis();
        long elapsedTime = now - sampleMillis;
        sampleMillis = now;
        long n = sampleCount;
        sampleCount = 0;
        Serial.print("samples/s "); Serial.println(n * 1000.0 / elapsedTime);
      }
    }

    delay(100);
  }
}

void loop() {
  unsigned int prevBits = *GPIOP;
  unsigned int bits;
  int change;
  int i;

  while (1) {
    // digitalWrite(2, 1);
    bits = *GPIOP;
    uRaw += changeTable[get2bits(uPhA, prevBits)][get2bits(uPhA, bits)];
    vRaw += changeTable[get2bits(vPhA, prevBits)][get2bits(vPhA, bits)];
    wRaw += changeTable[get2bits(wPhA, prevBits)][get2bits(wPhA, bits)];
    prevBits = bits;
    // digitalWrite(2, 0);
    sampleCount++;
  }
}

void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.begin(115200);
  while (!Serial) ;

  pinMode(uPhA, INPUT);
  pinMode(uPhB, INPUT);
  pinMode(vPhA, INPUT);
  pinMode(vPhB, INPUT);
  pinMode(wPhA, INPUT);
  pinMode(wPhB, INPUT);
  pinMode(16, OUTPUT);
  pinMode(2, OUTPUT);

  digitalWrite(16, 0);  // 1 enable encoder inputs, 0 disable inputs and let esp32 pins float

  wRaw = vRaw = uRaw = 0;

  Serial.print("arduino core ");
  Serial.println(xPortGetCoreID());

  xTaskCreatePinnedToCore(
    uiTask,    /* Task function. */
    "uiTask",  /* name of task. */
    1000,           /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &decoderTask,   /* Task handle to keep track of created task */
    0);             /* Core */
}
