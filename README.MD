![image](https://openfixtures.punchav.com/full_grey.png)
## Statement of Intent
This project is an Arduino based lighting fixture system that provides an easy and achievable to make a lighting fixture from scratch. This will be simple to use, and will be able to make complex fixtures, such as Dimmer channels, Relay channels, and later down the track, color and position controls. This will be great for the experimentalist lighting designer with a great idea for a custom fixture Martin won't make, or a cool DMX controlled thingamabob that Ayrton don't have planned yet.

### Roadmap and Objectives
**Note that this roadmap is non-linear and some tasks may appear completed before others. This may happen in part due to some things being in beta before being adopted, or because I can't be stuffed to do the next thing. Anything with the ~~strikethrough~~ has been done.**
1. Color control in many modes, such as CMY, ~~RGB, and integration to LED strip protocols such as WS2812B or NeoPixel.~~ *[UPDATE: Color is supported in RGB mode, as well as pixel/non-pixel WS2812B & NeoPixel]*
2. ~~Movement controls with Pan and/or Tilt.~~ This would theoretically be accompanied by Fine/Coarse modes for both channels, as this is a staple of lighting fixtures. *[UPDATE: Servo motors are implemented in a non-tested, purely beta method. Steppers coming soon.]*
3. ~~Custom fixture maker tool~~ that is able to export both OpenFixtures code, and .GDTF files, meaning the LD can rest easy integrating custom lights onto robust, existing controllers, such as MA, Eos, Onyx, and ChamSys. This support will not be extended to all controllers, but where applicable, I will definitely attempt to make sure these formats such as .GDTF will be useful. *[UPDATE: Python based code generator now exists: GTDF and other not yet, however they'll be here when I can get around to them.]*
4. WDMX support for generic WDMX receivers, such as WDMXTRXII boards from Event Lighting *[UPDATE: Not yet. Can't figure it out...]*
5. ~~PCB Support with a custom OpenFixtures PCB.~~ This would theoretically be many different models of fixtures that feature a FET-driver for 12V controls, a Servo driver, and maybe an RGB LED Controller. These would be the products associated with the project intended for a custom lighting fixture that can be reliable as well as repairable, customisable, and affordable. *[UPDATE: Version 1 of the PCB is in design and does exist in beta. First prototypes available soon.]*
6. Open Lighting Project support. This was pointed out by a member of [r/lightingdesign](https://reddit.com/lightingdesign) that OLP would be an efficient way to assist LD's to make and use OpenFixtures. It's on the roadmap now, hopefully a tool could be made within a few months, we'll see.

## Installation
**This guide is for Windows interations of the Arduino IDE, and may not bear any resemblence to the Mac OS or Linux system installation steps. I'm not a Mac person, and I have no intention to give any money to Apple; thus I am unable to properly check whether this runs properly on Mac OS. As for Linux, you nerds are probably smart enough to figure this shit out for yourselves.**

1. Download and install the Arduino IDE (https://www.arduino.cc/en/software)
2. Select a supported microcontroller (AVE, etc)
3. To install the library, download the latest build, and select it for installation:
- Sketch > Include Library > Add .ZIP library > *Select downloaded ZIP file*
4. Wait for installation to run.
5. To use a pre-made example fixture setup file, access the Examples with:
- File > Examples > OpenFixtures > *example to load*
6. Use Ctrl-U to upload to the board. Note that in most cases, the RS485 $\rightarrow$ Arduino connection must be removed while uploading to ensure the comms bus works properly. This can be avoided by using the Arduino Leonardo which has the seperate USB buses, but that is yet untested, so try it at your own discretion.

## How to build an OpenFixtures 4xFET
To build a prototype of the OpenFixtures beta (which is the 4xFET controller), download the zip folder in the PCB folder, upload it to a PCB service, and get it made. Sourcing parts from [Jaycar](https://jaycar.com.au), [Core Electronics](https://core-electronics.com.au), and [Element14](https://au.element14.com) was easy for an Australian, but find any reliable source for these parts.

*Note: Any model of Arduino Uno will **physically** fit on the PCB - I give no guarantee that it will work. I tested with the Arduino Uno Rev3, but I've spec'ed the Rev4 because it was cheaper from my local retailer.*
### Bill of Materials
| Item Name | Item Ref (From Manufacturer) | Quantity |
|-----------|------------------------------|----------|
|PCB|*Download from GitHub*|1|
|NC5FAH (Neutrik Connector, 5 Pins, Female, Right Angle, PCB)|020418 (Element14)|1|
|NC5MAH (Neutrik Connector, 5 Pins, Male, Right Angle, PCB)|8020396 (Element14)|1|
|NC3FAH (Neutrik Connector, 3 Pins, Female, Right Angle, PCB)|724518 (Element14)|1|
|NC3MAH (Neutrik Connector, 3 Pins, Male, Right Angle, PCB)|7244543 (Element14)|1|
|Arduino Uno R3 (R4 also supported)|XC9210 (Jaycar)|1|
|MAX485 TTL-UART|CE05154 (Core Electronics)|1|
|Screw Terminals (2-pin)|HM3172 (Jaycar)|6|
|Micro Slide Switch|SS0834 (Jaycar)|2|
|2.54mm Male Strip (40-pin)|HM3212 (Jaycar)|1|
|1N5408 3A Diode|ZR1018 (Jaycar)|2|
|LM7809 9V Regulator **(Ensure your Arduino accepts the voltage you feed it! Most will take 9V, however if you don't feed it 9V, it won't output ANY voltage, and thus you must use a VREG that outputs enough voltage for your MCU.)**|ZV1509 (Jaycar)|1|

### Board Specific Information
|Channel|Pin|
|--|--|
|Channel 1|11|
|Channel 2|10|
|Channel 3|9|
|Channel 4|6|
## How to code an OpenFixture
### Starting code
```
#include <OpenFixtures.h>

Fixture myFixture(1);

void setup() {
  fixture.begin();
}

void loop() {

}
```
### Adding a "function" to the Fixture
To add a function to the fixture, go to the fixture definition, as written below, and copy it *after* the `Fixture myFixture(1);` line, which instantiates the function.
Then, add the object of the function to the `setup` object, then add `.begin();` to begin the function object. After this, add the function object to the `loop` object, with `.refresh();` after this. This calls the function to refresh itself against the DMX signal. Upload the code to the board, and connect the DMX via an RS485 $\rightarrow$ TTL board.
## Examples
### Dimmer Channels
#### SimpleDimmer
```
Fixture Definition:
SimpleDimmer dimmer(1, 6);
1: Address
2: Arduino Pin (Ensure that you're using a PWM-enabled pin)
```

This code creates a single Dimmer Object called dimmer1 that takes the DMX address from a DMX address, and forwards the value onto a PWM pin. This is now a simple and straight-forward process for the maker.

#### Standard Strobe
```
Fixture Definition:
Strobe strobe(1, 6, 10, 1000, 1023, 0, true, true);
1: Address
2: Arduino Pin
3: Fastest (Period)
4: Slowest (Period)
5: Highest Value to write (Raw Arduino Value)
6: Lowest Value to write (Raw Arduino Value)
7: Lower boundary snap (Bool)
8: High boundary snap (Bool)
```

This code creates a strobe that strobes an Arduino pin that controls the pin between different strobing frequencies. The strobe's total intensity is controlled by the second address. These timings are in Period, so to convert:$\frac{1000}{\text{Hz}}=T\,\,\text{(Period)}$. When the DMX value is 0, it will disable the strobe effect. When lower and/or higher boundary are `true`, the DMX Values for low and high become a solid on/off.

#### Intensity Controlled Strobe
```
Fixture Definition:
IntStrobe strobe(1, 6, 10, 1000, 1023, 0, true, true);
1: Address
2: Arduino Pin
3: Fastest (Period)
4: Slowest (Period)
5: Highest Value to write (Raw Arduino Value)
6: Lowest Value to write (Raw Arduino Value)
7: Lower boundary snap (Bool)
8: High boundary snap (Bool)
```

This strobe is different, in that it will uses a different DMX control structure to strobe. The DMX control table is below:
| DMX Address (Relative to GlobalAddress) | Function                 |
|-----------------------------------------|--------------------------|
| 1                                       | Intensity at peak strobe |
| 2                                       | Strobe frequency         | 

When the DMX value is 0, it will disable the strobe effect. When lower and/or higher boundary are `true`, the DMX Values for low and high become a solid on/off.


#### RangedDimmer (**Currently a bit buggered**)
```
Fixture Definition:
RangedDimmer dimmer(1, 6, 0, 50, 0, 128);
1: Address
2: Arduino Pin (Ensure that you're using a PWM-enabled pin)
3: DMX Low (between 0 and 1023)
4: DMX High (between 0 and 1023)
5: Output Low (between 0 and 1023)
6: Output High (between 0 and 1023)
```
This is a ranged dimmer, meaning you can change the high and low range for both the DMX input and physical 10-bit output.
### Relay Channels
```
Fixture Definition:
Relay relay(1, 6, 128, false);
1: Address
2: Arduino Pin (Ensure that you're using a PWM-enabled pin)
3: Threshold
4: Logic inversion
```
This code creates a single Relay Object called relay1 that takes the DMX address from address 1, and forwards the value onto Digital Pin 6, provided that it's value is above or equal to 128/256, keeping in mind that the **inverted** flag is false; if it were true; this logic would be inverted. This is now a simple and straight-forward process for the maker. This doesn't have to be a relay, rather it can be any binary input device, such as an object you want to turn on instantly, such as motor with no speed control other than on/off.

### RGB
```
Fixture Definition:
RGB rgb(1, 6, 7, 8);
1: Address
2: Red Pin (Ensure that you're using a PWM-enabled pin)
3: Green Pin
4: Blue Pin
```
If you've followed along so far (good on you), you will be able to read this like it's English (oder Deutsch) now. That's the very point of this project. This is for RGB with no intensity channel in the DMX layout.

### IRGB
```
Fixture Definition:
IRGB irgb(1, 6, 7, 8);
1: Address
2: Red Pin (Ensure that you're using a PWM-enabled pin)
3: Green Pin
4: Blue Pin
```
Again, this is the IRGB fixture profile, with a very simple initialization alike the RGB profile. Great! This now has an intensity channel in the DMX that allows cheaper controllers to have better control of these fixtures.

### NeoPixel/WS2812/WS2812B (Pixel Mapped & RGB)
```
Fixture Definition:
NeoPixel_PM_RGB neo(1, 6, 8, 1);
1: Address
2: NeoPixel Pin
3: Number of Pixels
4: Starting pixel
```
This is where things both get funky and strange at the same time.
This is the NeoPixel and WS2812 class for Pixel Map RGB fixtures where you have many pixels in a strip. This is definitely the coolest fixture (yet) that will probably be the most used fixture type for DIY fixtures.

### NeoPixel/WS2812/WS2812B (**Non**-Pixel Mapped & RGB)
```
Fixture Definition:
NeoPixel_RGB neo(1, 6, 8, 1);
1: Address
2: NeoPixel Pin
3: Number of Pixels
4: Starting pixel
```
This is the same as the NeoPixel sketch as written above, however with the twist that this doesn't pixel map the cells of the NeoPixel, rather treats a strip as one fixture. It still requires a start and end pixel, so a strip can be broken up into a compound multi-fixture arrangement. It's very complex, but this is the most flexible option.

### Servo Motors
#### "Simple" Servo
```
Fixture Definition:
DMXServo servo(1, 6, 0, 180);
1: Address
2: Servo Pin
3: Servo Minimum
4: Servo Maximum
```
This is the control structure for controlling Servo Motors from OpenFixtures given a full range of the servo, where the administrator can change the full range of the servo in runtime. This allows for moving head/gobo/color wheel movement within OpenFixtures.

#### Servo Reel
```
Fixture Definition:
DMXServo servo(1, 6, 0, 180, 6, 0);
1: Address
2: Servo Pin
3: Servo Minimum
4: Servo Maximum
5: Number of "slots"
6: Adjustment angle
```
DMX controlled Servo that is broken up into a discrete steps, designed for gobos or gels, perhaps. This is helpful with the additon of the Adjustment Angle (essentially the $ + c $ , for the calculus folk).

### Fixture Controls
The `Fixture` object in the OpenFixtures project is able to control the fixture and it's parts, such as the DMX Address. You can do this with the following code:
#### `fixture.AddressAdd(value)`
- Adds `value` to the DMX Address. Good for making a UI that allows you to control the Address without needing to re-upload.
#### `fixture.AddressSet(value)`
- Sets `value` to the DMX Address, which is even easier for making a UI, which is very useful for creating the **entire** fixture, which is what OpenFixtures is all about.
#### `fixture.AddressGet()`
- Returns the current DMX Address. Again, useful for making a UI for the fixture, with perhaps a display, and EEPROM non-volatile memory.

## FixtureCreator
In this GitHub folder, you will see a python script in FixtureCreatorBeta/creator.py that gives you access  to the beta of the Arduino script creator for OpenFixtures. Experiment at your own risk. It's kinda cool I guess.

## Help out
Please do. It would be incredible if you forked the project to add any cool feature you want to see. I'm one person, and get busy quickly. Please consider publishing any code modifications or additions including new features you produce.

## License and Usage
### Can
- Use for free in development of a fixture or lighting system.
- Modify (with the asterisk that modifications must link to the original work and creator)
- Email for support, further questions, and praise (or more likely hate)
- Use in a commercial venue or in the processes of business, such as a freelance LD or theatre manager.
- Commercialize a product built on the OpenFixtures code or designs, with a reference to your use of this material, such as on a website or in an instructional manual.
### Can't
- Pretend it's yours.
- Use in a system with either the intent or ability to injure a person (within the bounds of practiability) such as flame generation, cannons, other dangerous or restricted methods. I'm not taking responsibility because you made a DMX controlled chainsaw and it took your leg off.
- Trust me to contribute regularly.
- Travel faster than $3.00 \times 10^{8} \text{ } \text{ms}^{-1}$.
### Please
- Send me an email about how you used my code/designs.
- Use responsibly and avoid being evil.
- Buy me a coffee, maybe?
- Star the project on GitHub
- Follow me on Instagram. @open_fixtures
