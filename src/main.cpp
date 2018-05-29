
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "esp_task_wdt.h"
#include <math.h>

const char xPhA = 25;
const char xPhB = 26;
const char yPhA = 12;
const char yPhB = 13;
const char zPhA = 14;
const char zPhB = 15;
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

long int u;   // angle between vertical pillar and arm b
long int v;   // angle between arm b and arm c
long int w;   // rotation around vertical pillar

esp_err_t show_esp_error(const char *title, esp_err_t result) {
  if (result != ESP_OK) {
    Serial.print("error ");
    if (result == ESP_ERR_NOT_FOUND) {
      Serial.println("ESP_ERR_NOT_FOUND");
    } else if (result == ESP_ERR_INVALID_STATE) {
      Serial.println("ESP_ERR_INVALID_STATE");
    } else {
      Serial.print(result);
    }
    Serial.print(" in: ");
    Serial.print(title);
  }
  return result;
}

void decoderLoop(void *parameter) {
  unsigned int prevBits = *GPIOP;
  unsigned int bits;
  int change;
  int i;

  while (1) {
    for (i = 0; i < 1000000; i++) {
      bits = *GPIOP;
      w += changeTable[get2bits(xPhA, prevBits)][get2bits(xPhA, bits)];
      v += changeTable[get2bits(yPhA, prevBits)][get2bits(yPhA, bits)];
      u += changeTable[get2bits(zPhA, prevBits)][get2bits(zPhA, bits)];
      prevBits = bits;
    }
    delay(1); // have to do this to keep the watchdog on the mat
  }
}



void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.begin(115200);
  while (!Serial) ;

  pinMode(xPhA, INPUT);
  pinMode(xPhB, INPUT);
  pinMode(yPhA, INPUT);
  pinMode(yPhB, INPUT);
  pinMode(zPhA, INPUT);
  pinMode(zPhB, INPUT);
  pinMode(16, OUTPUT);

  digitalWrite(16, 0);  // 1 enable encoder inputs, 0 disable inputs and let esp32 pins float

  w = v = u = 0;

  xTaskCreatePinnedToCore(
    decoderLoop,    /* Task function. */
    "decoderTask",  /* name of task. */
    1000,           /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &decoderTask,   /* Task handle to keep track of created task */
    0);             /* Core */

  Serial.println("ready!!");
}

typedef struct {
  float x, y, z;
} Point3D;


// measure
const int revSteps = 120000;
const int a = 200;
const int b = 200;
const int c = 200;
const float uZero = 90;
const float vZero = 90;
const float wZero = 0;
void calcXYZ(Point3D &p, long int uRaw, long int vRaw, long int wRaw) {
  float u = uRaw * M_PI * 2 / revSteps;
  float v = vRaw * M_PI * 2 / revSteps;
  float w = wRaw * M_PI * 2 / revSteps;
  p.y = (b * sin(u) - c * sin(u + v)) * cos(w);
  p.z = b * cos(u) - c * cos(u + v) - a;
  p.x = p.y * sin(w);
}

void printCoords() {
  Point3D p;
  calcXYZ(p, u, v, w);

  Serial.print(p.x);
  Serial.print(", ");
  Serial.print(p.y);
  Serial.print(", ");
  Serial.println(p.z);
}

void loop() {
  if (Serial.available() > 0) {
    printCoords();
    Serial.read();
  }
}
