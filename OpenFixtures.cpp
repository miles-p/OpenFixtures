#include "OpenFixtures.h"
void Fixture::begin() {
  globalAddress = addressPriv - 1;
  DMXSerial.init(DMXReceiver);
};

Fixture::Fixture(int address) {
  addressPriv = address;
};

// Method definition to initialize dimmer pin
void SimpleDimmer::begin() {
  pinMode(pinPriv, OUTPUT); // Set pin as output
}
// Constructor for Dimmer class
SimpleDimmer::SimpleDimmer(int address, int pin) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
}

void Strobe::begin() {
  pinMode(pinPriv, OUTPUT);
  statePriv = true;
  millisPrev = 0;
};

Strobe::Strobe(int address, int pin, int slowest, int fastest, int levelOn, int levelOff, bool lowOff, bool highOn) {
  addressPriv = address;
  pinPriv = pin;
  slowestPriv = slowest;
  fastestPriv = fastest;
  levelOnPriv = levelOn;
  levelOffPriv = levelOff;
  lowOffPriv = lowOff;
  highOnPriv = highOn;
};


void IntStrobe::begin() {
  pinMode(pinPriv, OUTPUT);
  statePriv = true;
  millisPrev = 0;
};

IntStrobe::IntStrobe(int address, int pin, int slowest, int fastest, int levelOn, int levelOff, bool lowOff, bool highOn) {
  addressPriv = address;
  pinPriv = pin;
  slowestPriv = slowest;
  fastestPriv = fastest;
  levelOnPriv = levelOn;
  levelOffPriv = levelOff;
  lowOffPriv = lowOff;
  highOnPriv = highOn;
};

// Method definition to initialize dimmer pin
void RangedDimmer::begin() {
  pinMode(pinPriv, OUTPUT); // Set pin as output
}

// Constructor for Dimmer class
RangedDimmer::RangedDimmer(int address, int pin, int dmxLow, int dmxHigh, int outputLow, int outputHigh) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
  dmxLowPriv = dmxLow; // Set DMX low range
  dmxHighPriv = dmxHigh; // Set DMX high range
  outputLowPriv = outputLow; // Set output low range
  outputHighPriv = outputHigh; // Set output high range
}

// Method definition to initialize relay pin
void Relay::begin() {
  pinMode(pinPriv, OUTPUT); // Set pin as output
};

// Constructor for Relay class
Relay::Relay(int address, int pin, int thresh, bool inverted) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
  threshPriv = thresh; // Set threshold
  invertedPriv = inverted; // Set inverted
};

// Method definition to initialize RGB LED pins
void RGB::begin() {
  pinMode(pinRPriv, OUTPUT); // Set red pin as output
  pinMode(pinGPriv, OUTPUT); // Set green pin as output
  pinMode(pinBPriv, OUTPUT); // Set blue pin as output
};

// Constructor for RGB class
RGB::RGB(int address, int pinR, int pinG, int pinB) {
  addressPriv = address; // Set DMX address
  pinRPriv = pinR; // Set red pin
  pinGPriv = pinG; // Set green pin
  pinBPriv = pinB; // Set blue pin
};

// Method definition to initialize individually controlled RGB LED pins
void IRGB::begin() {
  pinMode(pinRPriv, OUTPUT); // Set red pin as output
  pinMode(pinGPriv, OUTPUT); // Set green pin as output
  pinMode(pinBPriv, OUTPUT); // Set blue pin as output
};

// Constructor for IRGB class
IRGB::IRGB(int address, int pinR, int pinG, int pinB) {
  addressPriv = address; // Set DMX address
  pinRPriv = pinR; // Set red pin
  pinGPriv = pinG; // Set green pin
  pinBPriv = pinB; // Set blue pin
};

// Method definition to initialize NeoPixel RGB LEDs
void NeoPixel_PM_RGB::begin() {
  pixels = new Adafruit_NeoPixel(pixNumPriv, pinPriv, NEO_GRB + NEO_KHZ800); // Create NeoPixel object
  pixels->begin(); // Initialize NeoPixel strip
};

// Constructor for NeoPixel_PM_RGB class
NeoPixel_PM_RGB::NeoPixel_PM_RGB(int address, int pin, int pixNum, int startPix) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
  pixNumPriv = pixNum; // Set number of pixels
  startPixPriv = startPix; // Set start pixel
};

// Method definition to initialize NeoPixel RGB LEDs
void NeoPixel_RGB::begin() {
  pixels = new Adafruit_NeoPixel(pixNumPriv, pinPriv, NEO_GRB + NEO_KHZ800); // Create NeoPixel object
  pixels->begin(); // Initialize NeoPixel strip
};

// Constructor for NeoPixel_PM_RGB class
NeoPixel_RGB::NeoPixel_RGB(int address, int pin, int pixNum, int startPix) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
  pixNumPriv = pixNum; // Set number of pixels
  startPixPriv = startPix; // Set start pixel
};

// Method definition to initialize servo
void DMXServo::begin() {
  Hservo = new Servo(); // Create Servo object
  Hservo->attach(pinPriv); // Attach servo to pin
};

// Constructor for DMXServo class
DMXServo::DMXServo(int address, int pin, int servoMin, int servoMax) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
  servoMinPriv = servoMin; // Set minimum servo position
  servoMaxPriv = servoMax; // Set maximum servo position
};

// Method definition to initialize servo reel
void ServoReel::begin() {
  Rservo = new Servo(); // Create Servo object
  Rservo->attach(pinPriv); // Attach servo to pin
};

// Constructor for ServoReel class
ServoReel::ServoReel(int address, int pin, int servoMin, int servoMax, int reelSlotsCount, int adjustmentAngle) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
  servoMinPriv = servoMin; // Set minimum servo position
  servoMaxPriv = servoMax; // Set maximum servo position
  reelSlotsCountPriv = reelSlotsCount; // Set number of reel slots
  adjustmentAnglePriv = adjustmentAngle; // Set adjustment angle
};