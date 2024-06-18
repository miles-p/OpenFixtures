#include <OpenFixtures.h>   // Import my library
RangedDimmer dimmer1(1, 6, 0, 255, 0, 511);   // Address 1 and pin 6, take the normal DMX Input, but only range the output from 0 to 511 out of 1023.

void setup() {
DMXSerial.init(DMXReceiver);   // This bit is kinda magic and if you ignore it, everything works fine!
dimmer1.begin();   // Initialize the dimmer object.
}

void loop() {
dimmer1.refresh();
}