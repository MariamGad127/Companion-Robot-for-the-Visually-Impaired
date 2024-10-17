#include <Ultrasonic.h>
Ultrasonic uTop(A3, A4);
Ultrasonic uRight(6, 5);
Ultrasonic uFront(A1, A2);
Ultrasonic uLeft(2, 3);

int LeftDistance;
int RightDistance;
int FrontDistance;
int TopDistance;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read distance readings from all sides
  LeftDistance = uLeft.read();
  RightDistance = uRight.read();
  FrontDistance = uFront.read();
  TopDistance = uTop.read();
  
  Serial.println((String)"Front: " + FrontDistance + (String)"  Left: " + LeftDistance + (String)"  Right: " + RightDistance + (String)"  Top: " + TopDistance);
}
