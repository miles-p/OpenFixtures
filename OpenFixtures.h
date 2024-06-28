/*
  OpenFixtures - by Miles Punch
  An open-source framework for building intelligent lighting fixtures from scratch.
  GitHub Repository: https://github.com/miles-p/OpenFixtures
*/

#ifndef OpenFixtures
#define OpenFixtures
#endif

#include <DMXSerial.h> // Include DMXSerial library
#include <Adafruit_NeoPixel.h> // Include Adafruit NeoPixel library
#include <Servo.h> // Include Servo library
#include <AccelStepper.h> // Include AccelStepper library
#include "Arduino.h" // Include Arduino library

// Class for controlling dimmers
class SimpleDimmer {
  public:
    // Method to initialize dimmer
    // @brief Initialize all required parts of the dimmer object. 
    void begin();

    // @brief Method to update dimmer output based on DMX input
    void refresh() {
      analogWrite(pinPriv, DMXSerial.read(addressPriv)); // Write DMX value to pin
    }
    SimpleDimmer(int address, int pin); // Constructor
  private:
    int addressPriv;   // DMX address of the dimmer
    int pinPriv;        // Pin connected to the dimmer
};

// Method definition to initialize dimmer pin
void SimpleDimmer::begin() {
  pinMode(pinPriv, OUTPUT); // Set pin as output
};
// Constructor for Dimmer class
SimpleDimmer::SimpleDimmer(int address, int pin) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
};

// Class for controlling dimmers
class RangedDimmer {
  public:
    // Method to initialize dimmer
    // @brief Initialize all required parts of the dimmer object. 
    void begin();

    // @brief Method to update dimmer output based on DMX input
    void refresh() {
      analogWrite(pinPriv, map(DMXSerial.read(addressPriv), dmxLowPriv, dmxHighPriv, outputLowPriv, outputHighPriv)); // Write mapped DMX value to pin
    }
    RangedDimmer(int address, int pin, int dmxLow, int dmxHigh, int outputLow, int outputHigh); // Constructor
  private:
    int addressPriv;   // DMX address of the dimmer
    int pinPriv;        // Pin connected to the dimmer
    int dmxLowPriv;     // DMX low range
    int dmxHighPriv;    // DMX high range
    int outputLowPriv;  // Output low range
    int outputHighPriv; // Output high range
};

// Method definition to initialize dimmer pin
void RangedDimmer::begin() {
  pinMode(pinPriv, OUTPUT); // Set pin as output
};

// Constructor for Dimmer class
RangedDimmer::RangedDimmer(int address, int pin, int dmxLow, int dmxHigh, int outputLow, int outputHigh) {
  addressPriv = address; // Set DMX address
  pinPriv = pin; // Set pin
  dmxLowPriv = dmxLow; // Set DMX low range
  dmxHighPriv = dmxHigh; // Set DMX high range
  outputLowPriv = outputLow; // Set output low range
  outputHighPriv = outputHigh; // Set output high range
};

// Class for controlling relays
class Relay {
  public:
    // Method to initialize relay
    void begin();
    // Method to update relay output based on DMX input
    void refresh() {
      if (DMXSerial.read(addressPriv) >= threshPriv) { // Check if DMX value is above threshold
        digitalWrite(pinPriv, !invertedPriv); // Activate relay
      } else {
        digitalWrite(pinPriv, invertedPriv); // Deactivate relay
      };
    };
    Relay(int address, int pin, int thresh, bool inverted); // Constructor
  private:
    int addressPriv;   // DMX address of the relay
    int pinPriv;       // Pin connected to the relay
    int threshPriv;    // Threshold for relay activation
    bool invertedPriv; // Whether the relay is inverted
};

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

// Class for controlling RGB LEDs
class RGB {
  public:
    // Method to initialize RGB LED
    void begin();

    // Method to update RGB LED output based on DMX input
    void refresh() {
      analogWrite(pinRPriv, DMXSerial.read(addressPriv)); // Write DMX value to red pin
      analogWrite(pinGPriv, DMXSerial.read(addressPriv + 1)); // Write DMX value to green pin
      analogWrite(pinBPriv, DMXSerial.read(addressPriv + 2)); // Write DMX value to blue pin
    }
    RGB(int address, int pinR, int pinG, int pinB); // Constructor
  private:
    int addressPriv;   // DMX address of the RGB LED
    int pinRPriv;      // Red pin connected to the RGB LED
    int pinGPriv;      // Green pin connected to the RGB LED
    int pinBPriv;      // Blue pin connected to the RGB LED
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

// Class for controlling RGB LEDs with individual control
class IRGB {
  public:
    // Method to initialize individually controlled RGB LED
    void begin();

    // Method to update individually controlled RGB LED output based on DMX input
    void refresh() {
      analogWrite(pinRPriv * (DMXSerial.read(addressPriv)) / 256, DMXSerial.read(addressPriv + 1)); // Write scaled DMX value to red pin
      analogWrite(pinGPriv * (DMXSerial.read(addressPriv)) / 256, DMXSerial.read(addressPriv + 2)); // Write scaled DMX value to green pin
      analogWrite(pinBPriv * (DMXSerial.read(addressPriv)) / 256, DMXSerial.read(addressPriv + 3)); // Write scaled DMX value to blue pin
    }
    IRGB(int address, int pinR, int pinG, int pinB); // Constructor
  private:
    int addressPriv;   // DMX address of the individually controlled RGB LED
    int pinRPriv;      // Red pin connected to the RGB LED
    int pinGPriv;      // Green pin connected to the RGB LED
    int pinBPriv;      // Blue pin connected to the RGB LED
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

// Class for controlling NeoPixel RGB LEDs with Pixel Mapping
class NeoPixel_PM_RGB {
  public:
    // Method to initialize NeoPixel RGB LEDs
    void begin();

    // Method to update NeoPixel RGB LEDs output based on DMX input
    void refresh() {
      pixels->clear(); // Clear NeoPixel buffer
      for (int i = startPixPriv; i < pixNumPriv+startPixPriv; i++) { // Loop through pixels
        pixels->setPixelColor(i-1, pixels->Color(DMXSerial.read((3*i-2)+(addressPriv-1)), DMXSerial.read((3*i-1)+(addressPriv-1)), DMXSerial.read((3*i)+(addressPriv-1)))); // Set pixel color based on DMX values
      };
      pixels->show(); // Update NeoPixel strip
    };

    // Constructor for NeoPixel_PM_RGB class
    NeoPixel_PM_RGB(int address, int pin, int pixNum, int startPix); // Constructor
  private:
    int addressPriv;           // DMX address of the NeoPixel
    int pinPriv;               // Pin connected to the NeoPixel
    int pixNumPriv;            // Number of NeoPixels
    int startPixPriv;          // Start pixel
    Adafruit_NeoPixel* pixels; // NeoPixel object
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

// Class for controlling NeoPixel RGB LEDs with Pixel Mapping
class NeoPixel_RGB {
  public:
    // Method to initialize NeoPixel RGB LEDs
    void begin();

    // Method to update NeoPixel RGB LEDs output based on DMX input
    void refresh() {
      pixels->clear(); // Clear NeoPixel buffer
      for (int i = startPixPriv; i < pixNumPriv+startPixPriv; i++) { // Loop through pixels
        pixels->setPixelColor(i-1, pixels->Color(DMXSerial.read(1+(addressPriv-1)), DMXSerial.read(1+(addressPriv-1)), DMXSerial.read(1+(addressPriv-1)))); // Set pixel color based on DMX values
      };
      pixels->show(); // Update NeoPixel strip
    };

    // Constructor for NeoPixel_PM_RGB class
    NeoPixel_RGB(int address, int pin, int pixNum, int startPix); // Constructor
  private:
    int addressPriv;           // DMX address of the NeoPixel
    int pinPriv;               // Pin connected to the NeoPixel
    int pixNumPriv;            // Number of NeoPixels
    int startPixPriv;          // Start pixel
    Adafruit_NeoPixel* pixels; // NeoPixel object
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

// Class for controlling servos with DMX
class DMXServo {
  public:
    void begin();
    void refresh() {
      Hservo->write(map(DMXSerial.read(addressPriv),0,255,servoMinPriv,servoMaxPriv)); // Write mapped DMX value to servo
    };
    DMXServo(int address, int pin, int servoMin, int servoMax); // Constructor
  private:
    int addressPriv; // DMX address
    int pinPriv; // Pin connected to servo
    int servoMinPriv; // Minimum servo position
    int servoMaxPriv; // Maximum servo position
    Servo* Hservo; // Servo object
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

// Class for controlling servo reels with DMX
class ServoReel {
  public:
    void begin();
    void refresh() {
      Rservo->write(map(DMXSerial.read(addressPriv),0,255,0, reelSlotsCountPriv)*((servoMaxPriv-servoMinPriv)/reelSlotsCountPriv)+adjustmentAnglePriv); // Write mapped DMX value to servo reel
    };
    ServoReel(int address, int pin, int servoMin, int servoMax, int reelSlotsCount, int adjustmentAngle); // Constructor
  private:
    int addressPriv; // DMX address
    int pinPriv; // Pin connected to servo
    int servoMinPriv; // Minimum servo position
    int servoMaxPriv; // Maximum servo position
    int reelSlotsCountPriv; // Number of reel slots
    int adjustmentAnglePriv; // Adjustment angle
    Servo* Rservo; // Servo object
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

// Class for controlling stepper motors with DMX
class StepperMotor {
  public:
    void begin();
    void refresh() {
      int position = map(DMXSerial.read(addressPriv), 0, 255, minPositionPriv, maxPositionPriv); // Map DMX value to stepper position
      stepper->moveTo(position); // Move stepper to position
      stepper->run(); // Run stepper motor
    };
    StepperMotor(int address, int stepPin, int dirPin, int minPosition, int maxPosition, int maxSpeed, int acceleration); // Constructor
  private:
    int addressPriv; // DMX address
    int stepPinPriv; // Pin connected to step
    int dirPinPriv; // Pin connected to direction
    int minPositionPriv; // Minimum position
    int maxPositionPriv; // Maximum position
    AccelStepper* stepper; // Stepper motor object
};

// Method definition to initialize stepper motor
void StepperMotor::begin() {
  stepper = new AccelStepper(AccelStepper::DRIVER, stepPinPriv, dirPinPriv); // Create Stepper object
  stepper->setMaxSpeed(maxSpeedPriv); // Set maximum speed
  stepper->setAcceleration(accelerationPriv); // Set acceleration
};

// Constructor for StepperMotor class
StepperMotor::StepperMotor(int address, int stepPin, int dirPin, int minPosition, int maxPosition, int maxSpeed, int acceleration) {
  addressPriv = address; // Set DMX address
  stepPinPriv = stepPin; // Set step pin
  dirPinPriv = dirPin; // Set direction pin
  minPositionPriv = minPosition; // Set minimum position
  maxPositionPriv = maxPosition; // Set maximum position
  maxSpeedPriv = maxSpeed; // Set maximum speed
  accelerationPriv = acceleration; // Set acceleration
};
