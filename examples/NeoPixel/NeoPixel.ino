#include <OpenFixtures.h>   // Import my library
NeoPixel_PM_RGB neo(1, 6, 8, 1);   // Address 1, pin 6, 8 celled fixture, 1st cell onwards (this is not zero indexed)
Fixture fixture(1);

void setup() {
neo.begin();
fixture.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
neo.refresh();
}