#include <OpenFixtures.h>   // Import my library
Relay relay1(1, 6, 128, false);   // Address 1, pin 6, trigger threshold 128/256, turn ON at threshold
Fixture fixture(1);

void setup() {
  // put your setup code here, to run once:
relay1.begin();
fixture.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
relay1.refresh();
}