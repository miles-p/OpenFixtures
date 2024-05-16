/*
  OpenFixtures - by Miles Punch
  An open-source framework for building intelligent lighting fixtures from scratch.
  GitHub Repository: https://github.com/miles-p/OpenFixtures
*/

#ifndef OpenFixtures
#define OpenFixtures
#include <DMXSerial.h>
#include <Adafruit_NeoPixel.h>
#include "Arduino.h"
#endif

// Class for controlling dimmers
class Dimmer {
  public:
    // Method to initialize dimmer
    void begin();

    // Method to update dimmer output based on DMX input
    void refresh() {
      analogWrite(pinPriv, DMXSerial.read(addressPriv));
    }
    Dimmer(int address, int pin);
  private:
    int addressPriv;   // DMX address of the dimmer
    int pinPriv;        // Pin connected to the dimmer
};

// Method definition to initialize dimmer pin
void Dimmer::begin() {
  pinMode(pinPriv, OUTPUT);
};

// Constructor for Dimmer class
Dimmer::Dimmer(int address, int pin) {
  addressPriv = address;
  pinPriv = pin;
};

// Class for controlling relays
class Relay {
  public:
    // Method to initialize relay
    void begin();

    // Method to update relay output based on DMX input
    void refresh() {
      if (DMXSerial.read(addressPriv) >= threshPriv) {
        digitalWrite(pinPriv, !invertedPriv); // Activate relay
      } else {
        digitalWrite(pinPriv, invertedPriv);  // Deactivate relay
      }
    }
    Relay(int address, int pin, int thresh, bool inverted);
  private:
    int addressPriv;   // DMX address of the relay
    int pinPriv;       // Pin connected to the relay
    int threshPriv;    // Threshold for relay activation
    bool invertedPriv; // Whether the relay is inverted
};

// Method definition to initialize relay pin
void Relay::begin() {
  pinMode(pinPriv, OUTPUT);
};

// Constructor for Relay class
Relay::Relay(int address, int pin, int thresh, bool inverted) {
  addressPriv = address;
  pinPriv = pin;
  threshPriv = thresh;
  invertedPriv = inverted;
};

// Class for controlling RGB LEDs
class RGB {
  public:
    // Method to initialize RGB LED
    void begin();

    // Method to update RGB LED output based on DMX input
    void refresh() {
      analogWrite(pinRPriv, DMXSerial.read(addressPriv));
      analogWrite(pinGPriv, DMXSerial.read(addressPriv + 1));
      analogWrite(pinBPriv, DMXSerial.read(addressPriv + 2));
    }
    RGB(int address, int pinR, int pinG, int pinB);
  private:
    int addressPriv;   // DMX address of the RGB LED
    int pinRPriv;      // Red pin connected to the RGB LED
    int pinGPriv;      // Green pin connected to the RGB LED
    int pinBPriv;      // Blue pin connected to the RGB LED
};

// Method definition to initialize RGB LED pins
void RGB::begin() {
  pinMode(pinRPriv, OUTPUT);
  pinMode(pinGPriv, OUTPUT);
  pinMode(pinBPriv, OUTPUT);
};

// Constructor for RGB class
RGB::RGB(int address, int pinR, int pinG, int pinB) {
  addressPriv = address;
  pinRPriv = pinR;
  pinGPriv = pinG;
  pinBPriv = pinB;
}

// Class for controlling RGB LEDs with individual control
class IRGB {
  public:
    // Method to initialize individually controlled RGB LED
    void begin();

    // Method to update individually controlled RGB LED output based on DMX input
    void refresh() {
      analogWrite(pinRPriv * (DMXSerial.read(addressPriv)) / 256, DMXSerial.read(addressPriv + 1));
      analogWrite(pinGPriv * (DMXSerial.read(addressPriv)) / 256, DMXSerial.read(addressPriv + 2));
      analogWrite(pinBPriv * (DMXSerial.read(addressPriv)) / 256, DMXSerial.read(addressPriv + 3));
    }
    IRGB(int address, int pinR, int pinG, int pinB);
  private:
    int addressPriv;   // DMX address of the individually controlled RGB LED
    int pinRPriv;      // Red pin connected to the RGB LED
    int pinGPriv;      // Green pin connected to the RGB LED
    int pinBPriv;      // Blue pin connected to the RGB LED
};

// Method definition to initialize individually controlled RGB LED pins
void IRGB::begin() {
  pinMode(pinRPriv, OUTPUT);
  pinMode(pinGPriv, OUTPUT);
  pinMode(pinBPriv, OUTPUT);
};

// Constructor for IRGB class
IRGB::IRGB(int address, int pinR, int pinG, int pinB) {
  addressPriv = address;
  pinRPriv = pinR;
  pinGPriv = pinG;
  pinBPriv = pinB;
}

// Class for controlling NeoPixel RGB LEDs with Pixel Mapping
class NeoPixel_PM_RGB {
  public:
    // Method to initialize NeoPixel RGB LEDs
    void begin();

    // Method to update NeoPixel RGB LEDs output based on DMX input
    void refresh() {
      pixels->clear();
      for (int i = 1; i < pixNumPriv+1; i++) {
        pixels->setPixelColor(i-1, pixels->Color(DMXSerial.read((3*i-2)+3*(addressPriv-1)), DMXSerial.read((3*i-1)+3*(addressPriv-1)), DMXSerial.read((3*i)+3*(addressPriv-1))));
      };
      pixels->show();
    };

    // Constructor for NeoPixel_PM_RGB class
    NeoPixel_PM_RGB(int address, int pin, int pixNum);
  private:
    int addressPriv;           // DMX address of the NeoPixel
    int pinPriv;               // Pin connected to the NeoPixel
    int pixNumPriv;            // Number of NeoPixels
    Adafruit_NeoPixel* pixels; // NeoPixel object
};

// Method definition to initialize NeoPixel RGB LEDs
void NeoPixel_PM_RGB::begin() {
  pixels = new Adafruit_NeoPixel(pixNumPriv, pinPriv, NEO_GRB + NEO_KHZ800);
  pixels->begin();
};

// Constructor for NeoPixel_PM_RGB class
NeoPixel_PM_RGB::NeoPixel_PM_RGB(int address, int pin, int pixNum) {
  addressPriv = address;
  pinPriv = pin;
  pixNumPriv = pixNum;
};