#include <Arduino.h>

char encoderChange(char a, char b, char &prevSwitches);

long int x, y, z;
long outTime = 0;
const long OUT_INTERVAL_MS = 1000;
char prevBits;

void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  x = y = z = 0;

  prevBits = PIND;
}

void printCoords() {
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
}


static char changeTable[4][4] = {
 //00  01  10  11 - prevSwitches
  { 0,  1, -1,  0}, //00 - switches
  {-1,  0,  0,  1}, //01
  { 1,  0,  0, -1}, //10
  { 0, -1,  1,  0}  //11
};


void loop() {
  char bits = PIND;
  x -=  changeTable[(prevBits >> 2) & 3][(bits >> 2) & 3];
  y -=  changeTable[(prevBits >> 4) & 3][(bits >> 4) & 3];
  z -=  changeTable[(prevBits >> 6) & 3][(bits >> 6) & 3];
  prevBits = bits;

  if (Serial.available() > 0) {
    printCoords();
    Serial.read();
  }


  // // print x, y, z each second
  // long t = millis();
  // if (t > outTime + OUT_INTERVAL_MS) {
  //   outTime = t;
  //   printCoords();
  // }
}
