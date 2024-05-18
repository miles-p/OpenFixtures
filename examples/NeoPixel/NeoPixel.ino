#include <OpenFixtures.h>   // Import my library
NeoPixel_PM_RGB neo(1, 6, 8, 1);   // Address 1, pin 6, 8 celled fixture, 1st cell onwards (this is not zero indexed)

void setup() {
  // put your setup code here, to run once:
DMXSerial.init(DMXReceiver);
neo.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
neo.refresh();
}