## Statement of Intent
This project is an Arduino based lighting fixture system that provides an easy and achievable to make a lighting fixture from scratch. This will be simple to use, and will be able to make complex fixtures, such as Dimmer channels, Relay channels, and later down the track, color and position controls. This will be great for the experimentalist lighting designer with a great idea for a custom fixture Martin won't make, or a cool DMX controlled thingamabob that Ayrton don't have planned yet.

### Roadmap and Objectives
1. Color control in many modes, such as CMY, RGB, and integration to LED strip protocols such as WS2812B or NeoPixel.
2. Movement controls with Pan and/or Tilt. This would theoretically be accompanied by Fine/Coarse modes for both channels, as this is a staple of lighting fixtures.
3. Custom fixture maker tool that is able to export both OpenFixtures code, and .GDTF files, meaning the LD can rest easy integrating custom lights onto robust, existing controllers, such as MA, Eos, Onyx, and ChamSys.
4. WDMX support for generic WDMX receivers, such as WDMXTRXII boards from Event Lighting
5. PCB Support with a custom OpenFixtures PCB. This would theoretically feature a FET-driver for 12V controls, a Servo driver, and maybe an RGB LED Controller. This would be the product associated with the project intended for a custom lighting fixture that can be reliable.

## Examples
### Dimmer Channels
```
#include "OpenFixtures.h"

Dimmer dimmer1;

  

void setup() {

  // put your setup code here, to run once:

DMXSerial.init(DMXReceiver);

pinMode(6,OUTPUT);

  

dimmer1.begin(1,6);

}

  

void loop() {

  // put your main code here, to run repeatedly:

dimmer1.refresh();

}
```

This code creates a single Dimmer Object called dimmer1 that takes the DMX address from address 1, and forwards the value onto Digital Pin 6. This is now a simple and straight-forward process for the maker.
### Relay Channels
```
#include "OpenFixtures.h"

Relay relay1;

  

void setup() {

  // put your setup code here, to run once:

DMXSerial.init(DMXReceiver);

pinMode(6,OUTPUT);

  

relay1.begin(1, 6, 64, false);

}

  

void loop() {

  // put your main code here, to run repeatedly:

relay1.refresh();

}
```

This code creates a single Relay Object called relay1 that takes the DMX address from address 1, and forwards the value onto Digital Pin 6, provided that it's value is above or equal to 64/256, keeping in mind that the **inverted** flag is false; if it were true; this logic would be inverted. This is now a simple and straight-forward process for the maker. This doesn't have to be a relay, rather it can be any binary input device.