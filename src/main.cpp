#include <Arduino.h>

char encoderChange(char a, char b, char &prevSwitches);

long int x, y, z;
long outTime = 0;
const long OUT_INTERVAL_MS = 1000;

void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  x = y = z = 0;
}

char prevSwitches[3] = {-1, -1, -1};

void loop() {
  x += encoderChange(digitalRead(2), digitalRead(3), prevSwitches[0]);
  y += encoderChange(digitalRead(4), digitalRead(5), prevSwitches[1]);
  z += encoderChange(digitalRead(6), digitalRead(7), prevSwitches[2]);

    long t = millis();
    if (t > outTime + OUT_INTERVAL_MS) {
      outTime = t;
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.print(z);
      Serial.print("\n");
    }
}

// call continously with the two phases A, B of the quadrature encoder
// returns the change to the position based on any quadrature switch change
char encoderChange(char a, char b, char &prevSwitches) {
  // position change based on [fromSwitches][toSwitches]
  // where switches is two bits AB
  static char changeTable[4][4] = {
   //00  01  10  11 - prevSwitches
    { 0,  1, -1,  0}, //00 - switches
    {-1,  0,  0,  1}, //01
    { 1,  0,  0, -1}, //10
    { 0, -1,  1,  0}  //11
  };

  char switches = (a << 1) + b;
  if (prevSwitches == -1) {
    prevSwitches = switches;
  }
  char change = changeTable[prevSwitches][switches];
  prevSwitches = switches;
  return change;
}
