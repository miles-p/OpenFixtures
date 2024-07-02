#include <OpenFixtures.h>   // Import my library
RGB rgb1(1, 6, 7, 8);   // Address 1, pin 6-8 (for Red, Green, Blue)
Fixture fixture(1);
// DANGER: only use arduino pins that fully support PWM so that you have a fun time with colour dimming.

void setup() {
  // put your setup code here, to run once:
DMXSerial.init(DMXReceiver);
rgb1.begin();
fixture.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
rgb1.refresh();
}