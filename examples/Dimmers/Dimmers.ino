#include <OpenFixtures.h>       // Import my library
Dimmer dimmer1(1, 6);           // Define DMX address and pin: address 1 and pin 6

void setup() {
DMXSerial.init(DMXReceiver);    // This bit is kinda magic and if you ignore it, everything works fine!
dimmer1.begin();                // Initialize the dimmer object.
}

void loop() {
dimmer1.refresh();              // Refresh the dimmer object, only run this as often as you want to; broken up by a delay(timeInMilliseconds);
}

// That's it!