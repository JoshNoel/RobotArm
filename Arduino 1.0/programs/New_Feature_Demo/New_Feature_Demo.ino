#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition  = -999;
int oldForce = 0;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print("Position: ");
    Serial.println(newPosition);
  }
  if (digitalRead(6) == HIGH) {
    myEnc.write(0);
  }
  
  int newForce = analogRead(A0);
  if (newForce != oldForce) {
    oldForce = newForce;
    Serial.print("Force: ");
    Serial.println(newForce);
  }
}
