#include <OpenFixtures.h>
RGB rgb1;

void setup() {
  // put your setup code here, to run once:
  DMXSerial.init(DMXReceiver);
  pinMode(6, OUTPUT);

  rgb1.begin(1, 6, 7, 8); // Assuming pin 6 for Red, pin 7 for Green, and pin 8 for Blue
}

void loop() {
  // put your main code here, to run repeatedly:
  rgb1.refresh();
}
