#include <DMXSerial.h>

class Dimmer {
  public:
    void refresh() {
      analogWrite(pinPriv, DMXSerial.read(addressPriv));
    }
    void begin(int address, int pin) {
      addressPriv = address;
      pinPriv = pin;
    }
  private:
    int addressPriv;
    int pinPriv;
};