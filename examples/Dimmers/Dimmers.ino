#include <OpenFixtures.h>   // Import my library
Dimmer dimmer1(1, 6);   // Address 1 and pin 6

void setup() {
DMXSerial.init(DMXReceiver);   // This bit is kinda magic and if you ignore it, everything works fine!
dimmer1.begin();   // Initialize the dimmer object.
}

void loop() {
dimmer1.refresh();
}