#include <OpenFixtures.h>
Strobe strobe(1, 6, 10, 1000, 1023, 0, true, true);
Fixture fixture(1);

void setup() {
  // put your setup code here, to run once:

  strobe.begin();
  fixture.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  strobe.refresh();
}
