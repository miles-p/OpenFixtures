#include <OpenFixtures.h>   // Import my library
DMXServo dmxservo(1, 6, 0, 180);   // Address 1, pin 6, angle range 0>180 (degrees)
Fixture fixture(1);

void setup() {
  // put your setup code here, to run once:
dmxservo.begin();
fixture.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
dmxservo.refresh();
}