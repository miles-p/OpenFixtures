/*
  OpenFixtures - by Miles Punch
  An open source framework for building intelligent lighting fixtures, from scratch!
  https://github.com/miles-p/OpenFixtures
*/
#ifndef OpenFixtures
#define OpenFixtures
#include <DMXSerial.h>
#include <Adafruit_NeoPixel.h>
#include "Arduino.h"
#endif

// Dimmer class for controlling dimmer channels
class Dimmer {
  public:
    // Refresh method to update dimmer output based on DMX input
    void refresh() {
      analogWrite(pinPriv, DMXSerial.read(addressPriv));
    }
    // Initialization method for setting up the dimmer
    void begin(int address, int pin) {
      addressPriv = address;
      pinPriv = pin;
    }
  private:
    int addressPriv; // DMX address of the dimmer
    int pinPriv;     // Pin connected to the dimmer output
};

// Relay class for controlling relay channels
class Relay {
  public:
    // Refresh method to update relay output based on DMX input
    void refresh() {
      // Control relay based on DMX value and threshold
      if (DMXSerial.read(addressPriv) >= threshPriv) {
        digitalWrite(pinPriv, !invertedPriv); // Activate relay
      } else {
        digitalWrite(pinPriv, invertedPriv);  // Deactivate relay
      }
    }
    // Initialization method for setting up the relay
    void begin(int address, int pin, int threshold, bool inverted) {
      addressPriv = address;
      pinPriv = pin;
      threshPriv = threshold;
      invertedPriv = inverted;
    }
  private:
    int addressPriv;   // DMX address of the relay
    int pinPriv;       // Pin connected to the relay
    int threshPriv;    // Threshold value for relay activation
    bool invertedPriv; // Boolean indicating whether the relay logic is inverted
};

// Simple RGB class for controlling colour channels with no intensity parameter. For IRGB, use the IRGB class.
class RGB {
  public:
    // Refresh method to update relay output based on DMX input
    void refresh() {
      analogWrite(pinRPriv, DMXSerial.read(addressPriv));
      analogWrite(pinGPriv, DMXSerial.read(addressPriv+1));
      analogWrite(pinBPriv, DMXSerial.read(addressPriv+2));
    }
    // Initialization method for setting up the relay
    void begin(int address, int pinR, int pinG, int pinB) {
      addressPriv = address;
      pinRPriv = pinR;
      pinGPriv = pinG;
      pinBPriv = pinB;
    }
  private:
    int addressPriv;   // DMX address of the LED
    int pinRPriv;       // Pin connected to the LED
    int pinGPriv;       // Pin connected to the LED
    int pinBPriv;       // Pin connected to the LED
};

// Simple RGB class for controlling colour channels with no intensity parameter. For IRGB, use the IRGB class.
class IRGB {
  public:
    // Refresh method to update relay output based on DMX input
    void refresh() {
      analogWrite(pinRPriv*(DMXSerial.read(addressPriv)/1024), DMXSerial.read(addressPriv+1));
      analogWrite(pinGPriv*(DMXSerial.read(addressPriv)/1024), DMXSerial.read(addressPriv+2));
      analogWrite(pinBPriv*(DMXSerial.read(addressPriv)/1024), DMXSerial.read(addressPriv+3));
    }
    // Initialization method for setting up the relay
    void begin(int address, int pinR, int pinG, int pinB) {
      addressPriv = address;
      pinRPriv = pinR;
      pinGPriv = pinG;
      pinBPriv = pinB;
    }
  private:
    int addressPriv;   // DMX address of the LED
    int pinRPriv;       // Pin connected to the LED
    int pinGPriv;       // Pin connected to the LED
    int pinBPriv;       // Pin connected to the LED
};

// Let's get shit interesting! NeoPixel/WS2812/WS2812B compatibility for support with Pixel Mapping
class NeoPixel_PM_RGB {
  public:
    // Refresh method to update LED output based on DMX input
    void begin();

    void NeoPixel_PM_RGB::refresh() {
      pixels->clear();
      for(int i = 0; i<pixNumPriv; i++) {
        pixels->setPixelColor(i, pixels->Color(DMXSerial.read((3*(i+1))-2),DMXSerial.read((3*(i+1))-1),DMXSerial.read(3*(i+1))));
        //pixels->setPixelColor(1,pixels->Color(255,0,0));
      }
      pixels->show();
    }

    NeoPixel_PM_RGB(int address, int pin, int pixNum);
    // Initialization method for setting up the NeoPixels
  private:
    int addressPriv;   // DMX address of the NeoPixel
    int pinPriv;
    int pixNumPriv;
    Adafruit_NeoPixel* pixels;
};
void NeoPixel_PM_RGB::begin() {
  pixels = new Adafruit_NeoPixel(pixNumPriv, pinPriv, NEO_GRB + NEO_KHZ800);
  pixels->begin();
};
NeoPixel_PM_RGB::NeoPixel_PM_RGB(int address, int pin, int pixNum) {
  addressPriv = address;
  pinPriv = pin;
  pixNumPriv = pixNum;
}