
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

TaskHandle_t uiTaskHandle;
TaskHandle_t displayTaskHandle;

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

// void display_text(String text){
//   display.setColor(WHITE);
//   //display.setFont(Dialog_plain_40);
//   display.setTextAlignment(TEXT_ALIGN_CENTER);
//   display.drawString(64, 15, text);
//   display.display();
// }

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

void displayTask(void *parameter) {
  SSD1306 display(0x3c, 5, 4);
  char buf[100];
  Point3D p;

  display.init();
  display.clear();
  // display_text("ready.");
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_16);

  while (1) {
    display.clear();

    calcXYZ(p, uRaw, vRaw, wRaw);

    int yPos = 5;
    sprintf(buf, "%5.1f", p.x);
    display.drawString(30, yPos, "X");
    display.drawString(90, yPos, buf);
    yPos += 15;
    sprintf(buf, "%5.1f", p.y);
    display.drawString(30, yPos, "Y");
    display.drawString(90, yPos, buf);
    yPos += 15;
    sprintf(buf, "%5.1f", p.z);
    display.drawString(30, yPos, "Z");
    display.drawString(90, yPos, buf);

    display.display();

    delay(200);
  }

}

void loop() {
  unsigned int prevBits = *GPIOP;
  unsigned int bits;
  int change;
  int i;

  delay(2000);
  pinMode(0, OUTPUT);
  digitalWrite(0, 0);

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
  pinMode(0,INPUT);
  digitalWrite(0,HIGH);

  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

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
//  pinMode(encoderEnable, OUTPUT);

  Serial.print("arduino core ");
  Serial.println(xPortGetCoreID());


//  digitalWrite(encoderEnable, 0);  // 1 enable encoder inputs, 0 disable inputs and let esp32 pins float
  wRaw = vRaw = uRaw = 0;

  // displayTask(NULL);

  xTaskCreatePinnedToCore(
    uiTask,    /* Task function. */
    "uiTask",  /* name of task. */
    1000,           /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &uiTaskHandle,   /* Task handle to keep track of created task */
    0);             /* Core */

  xTaskCreatePinnedToCore(
    displayTask,    /* Task function. */
    "displayTask",  /* name of task. */
    5000,           /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &displayTaskHandle,   /* Task handle to keep track of created task */
    0);             /* Core */
}
