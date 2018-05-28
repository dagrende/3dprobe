
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "esp_task_wdt.h"

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

long x, y, z;

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
      x += changeTable[get2bits(xPhA, prevBits)][get2bits(xPhA, bits)];
      y += changeTable[get2bits(yPhA, prevBits)][get2bits(yPhA, bits)];
      z += changeTable[get2bits(zPhA, prevBits)][get2bits(zPhA, bits)];
      prevBits = bits;
    }
    delay(1);
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

  digitalWrite(16, 0);

  x = y = z = 0;

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

void printCoords() {
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
}

void loop() {
  if (Serial.available() > 0) {
    printCoords();
    Serial.read();
  }
}
