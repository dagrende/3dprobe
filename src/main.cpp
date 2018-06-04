
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "esp_task_wdt.h"
#include <math.h>
#include "SSD1306.h"

const char uPhA = 25;   // x encoder
const char uPhB = 26;
const char vPhA = 12;
const char vPhB = 13;
const char wPhA = 14;
const char wPhB = 15;
const char encoderEnable = 0;

#define get2bits(startBit, value) ((unsigned char)((value >> startBit) & 3))
const volatile int* GPIOP = (volatile int *)GPIO_IN_REG;

const static int changeTable[4][4] = {
 //00  01  10  11 - prevSwitches
  { 0,  1, -1,  0}, //00 - switches
  {-1,  0,  0,  1}, //01
  { 1,  0,  0, -1}, //10
  { 0, -1,  1,  0}  //11
};

// measures and angles
const int revSteps = 120000;    // encoder steps per revolution
const int a = 255 + 6.7;        // pillar base to u rotational center [mm]
const int b = 212;              // arm b length [mm]
const int c = 232 - 17.13 / 2;  // arm c length [mm]
const float uZero = 90 * M_PI / 180;
const float vZero = 90 * M_PI / 180;
const float wZero = 0;

typedef struct {
  float x, y, z;
} Point3D;

TaskHandle_t uiTaskHandle;
TaskHandle_t displayTaskHandle;

volatile long int uRaw;   // angle between vertical pillar and arm b
volatile long int vRaw;   // angle between arm b and arm c
volatile long int wRaw;   // rotation around vertical pillar

// vars for calculating sample frequency
volatile long sampleCount = 0;
long sampleMillis = 0;

// calculate p, cartesian coordinates from raw encoder angles
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

// print cartesian coodinates from raw encoder angles
void printCoords() {
  Point3D p;

  calcXYZ(p, uRaw, vRaw, wRaw);

  Serial.print(p.x);
  Serial.print(", ");
  Serial.print(p.y);
  Serial.print(", ");
  Serial.println(p.z);
}

// print raw encoder angles
void printRawAngles() {
  Serial.print(uRaw);
  Serial.print(", ");
  Serial.print(vRaw);
  Serial.print(", ");
  Serial.println(wRaw);
}

// read command char from serial, execute command and print to serial
void uiTask(void *parameter) {
  while (1) {
    if (Serial.available() > 0) {
      char ch = Serial.read();
      if (ch == 'p') {        // print current position
        printCoords();
      } else if (ch == 'a') { // print angles
        printRawAngles();
      } else if (ch == 'c') { // clear raw angles
        wRaw = vRaw = uRaw = 0;
      } else if (ch == 's') { // print average sample frequency
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

// continously display current x, y, z position
void displayTask(void *parameter) {
  SSD1306 display(0x3c, 5, 4);
  char buf[100];
  Point3D p;

  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_24);

  while (1) {
    display.clear();

    calcXYZ(p, uRaw, vRaw, wRaw);

    int yPos = 0;
    sprintf(buf, "%5.1f", p.x);
    display.drawString(25, yPos, "X");
    display.drawString(110, yPos, buf);
    yPos += 20;
    sprintf(buf, "%5.1f", p.y);
    display.drawString(25, yPos, "Y");
    display.drawString(110, yPos, buf);
    yPos += 20;
    sprintf(buf, "%5.1f", p.z);
    display.drawString(25, yPos, "Z");
    display.drawString(110, yPos, buf);

    display.display();

    delay(200);
  }
}

// continously sample encoder phase signals and step uRaw, vRaw, wRaw up/down
void loop() {
  unsigned int prevBits = *GPIOP;
  unsigned int bits;
  int change;
  int i;

  delay(2000);

  pinMode(0, OUTPUT);
  digitalWrite(0, 0); // enable encoder signal input buffer

  while (1) {
    bits = *GPIOP;
    uRaw += changeTable[get2bits(uPhA, prevBits)][get2bits(uPhA, bits)];
    vRaw += changeTable[get2bits(vPhA, prevBits)][get2bits(vPhA, bits)];
    wRaw += changeTable[get2bits(wPhA, prevBits)][get2bits(wPhA, bits)];
    prevBits = bits;
    sampleCount++;
  }
}


void setup() {
  // enable display
  pinMode(0,INPUT);
  digitalWrite(0,HIGH);

  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

  // disable WiFi and BLE
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

  wRaw = vRaw = uRaw = 0;

  // start ui task
  xTaskCreatePinnedToCore(
    uiTask,    /* Task function. */
    "uiTask",  /* name of task. */
    1000,           /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &uiTaskHandle,   /* Task handle to keep track of created task */
    0);             /* Core */

  // start display task
  xTaskCreatePinnedToCore(
    displayTask,    /* Task function. */
    "displayTask",  /* name of task. */
    5000,           /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &displayTaskHandle,   /* Task handle to keep track of created task */
    0);             /* Core */
}
