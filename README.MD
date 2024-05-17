![image](full_grey.png)
## Statement of Intent
This project is an Arduino based lighting fixture system that provides an easy and achievable to make a lighting fixture from scratch. This will be simple to use, and will be able to make complex fixtures, such as Dimmer channels, Relay channels, and later down the track, color and position controls. This will be great for the experimentalist lighting designer with a great idea for a custom fixture Martin won't make, or a cool DMX controlled thingamabob that Ayrton don't have planned yet.

### Roadmap and Objectives
**Note that this roadmap is non-linear and some tasks may appear completed before others. This may happen in part due to some things being in beta before being adopted, or because I can't be stuffed to do the next thing.**
1. Color control in many modes, such as CMY, RGB, and integration to LED strip protocols such as WS2812B or NeoPixel.
2. Movement controls with Pan and/or Tilt. This would theoretically be accompanied by Fine/Coarse modes for both channels, as this is a staple of lighting fixtures.
3. Custom fixture maker tool that is able to export both OpenFixtures code, and .GDTF files, meaning the LD can rest easy integrating custom lights onto robust, existing controllers, such as MA, Eos, Onyx, and ChamSys.
4. WDMX support for generic WDMX receivers, such as WDMXTRXII boards from Event Lighting
5. PCB Support with a custom OpenFixtures PCB. This would theoretically feature a FET-driver for 12V controls, a Servo driver, and maybe an RGB LED Controller. This would be the product associated with the project intended for a custom lighting fixture that can be reliable as well as repairable, customisable, and affordable.

## Examples
### Dimmer Channels
```
#include <OpenFixtures.h>   // Import my library
Dimmer dimmer1(1, 6);   // Address 1 and pin 6

void setup() {
DMXSerial.init(DMXReceiver);   // This bit is kinda magic and if you ignore it, everything works fine!
dimmer1.begin();   // Initialize the dimmer object.
}

void loop() {
dimmer1.refresh();
}
```

This code creates a single Dimmer Object called dimmer1 that takes the DMX address from address 1, and forwards the value onto Digital Pin 6. This is now a simple and straight-forward process for the maker.
### Relay Channels
```
#include <OpenFixtures.h>   // Import my library
Relay relay1(1, 6, 128, false);   // Address 1, pin 6, trigger threshold 128/256, turn ON at threshold

void setup() {
  // put your setup code here, to run once:
DMXSerial.init(DMXReceiver);
relay1.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
relay1.refresh();
}

```

This code creates a single Relay Object called relay1 that takes the DMX address from address 1, and forwards the value onto Digital Pin 6, provided that it's value is above or equal to 128/256, keeping in mind that the **inverted** flag is false; if it were true; this logic would be inverted. This is now a simple and straight-forward process for the maker. This doesn't have to be a relay, rather it can be any binary input device, such as an object you want to turn on instantly, such as motor with no speed control other than on/off.

### RGB
```
#include <OpenFixtures.h>   // Import my library
RGB rgb1(1, 6, 7, 8);   // Address 1, pin 6-8 (for Red, Green, Blue)
// DANGER: only use arduino pins that fully support PWM so that you have a fun time with colour dimming.

void setup() {
  // put your setup code here, to run once:
DMXSerial.init(DMXReceiver);
rgb1.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
rgb1.refresh();
}

```
If you've followed along so far (good on you), you will be able to read this like it's English now. That's the very point of this project. This is for RGB with no intensity channel in the DMX.
## License and Usage
### Can
- Use for free in development of a fixture or lighting system.
- Modify (with the asterisk that modifications must link to the original work and creator)
- Email for support, further questions, and praise (or more likely hate)
- Use in a commercial venue or in the processes of business, such as a freelance LD or theatre manager.
- Commercialize a product built on the OpenFixtures code or designs, with a reference to your use of this material, such as on a website or in an instructional manual.
### Can't
- Pretend it's yours.
- Use in a system with either the intent or ability to injure a person (within the bounds of practiability) such as flame generation, cannons, other dangerous or restricted methods. I'm not taking responsibility because you made a DMX controlled chainsaw and it took your face off.
- Trust me to contribute regularly. Ain't nobody got time for that.
- Travel faster than $3.00 \times 10^{8} \text{ms}^{-1}$.
### Please
- Send me an email about how you used my code/designs.
- Use responsibly and avoid being evil.
- Buy me a coffee, maybe?