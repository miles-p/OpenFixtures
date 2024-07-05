#include <OpenFixtures.h>
IRGB irgb(1, 5, 6, 7);
Fixture fixture(1);

void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  rgb1.begin(); // Assuming pin 6 for Red, pin 7 for Green, and pin 8 for Blue
  fixture.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  rgb1.refresh();
}
