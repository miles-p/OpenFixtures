#include <OpenFixtures.h>
NeoPixel_PM_RGB neopixel1(1, 6, 1);

void setup() {
  // put your setup code here, to run once:
  DMXSerial.init(DMXReceiver);

  neopixel1.begin(); // Assuming pin 6 for Red, pin 7 for Green, and pin 8 for Blue
}

void loop() {
  // put your main code here, to run repeatedly:
  neopixel1.refresh();
  //delay(300);
}
