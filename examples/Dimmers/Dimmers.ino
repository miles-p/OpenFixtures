#include <OpenFixtures.h>
Dimmer dimmer1;

void setup() {
  // put your setup code here, to run once:
DMXSerial.init(DMXReceiver);
pinMode(6,OUTPUT);

dimmer1.begin(1,6);
}

void loop() {
  // put your main code here, to run repeatedly:
dimmer1.refresh();
}