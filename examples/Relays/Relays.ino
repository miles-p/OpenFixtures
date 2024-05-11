#include <OpenFixtures.h>
Relay relay1;

void setup() {
  // put your setup code here, to run once:
DMXSerial.init(DMXReceiver);
pinMode(6,OUTPUT);

relay1.begin(1, 6, 64, false);
}

void loop() {
  // put your main code here, to run repeatedly:
relay1.refresh();
}
