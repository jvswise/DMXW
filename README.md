# DMXW
DMX-512 wireless gateway to networked Moteino RFM69 (Arduino compatible) based slave devices suitable for running battery operated stage effects up to 300 metres from the gateway.

The gateway allows DMX-512 channels to be be redistributed to low-cost, customizable, battery-operated Moteino based slave nodes. Slaves have 4 analog (PWM) channels that can drive several amps each at 5 -12 Vdc. They have an additional two digital channels. 

There is prototype code, schematics, PCB files, and user manuals:
  - A configurable DMX-512 to DMXW gateway with optional manaul override
    controls (joystick, toggle switches, and potentiometers).
  - A generic remote slave node.
  - A specialized remote slave node for driving an LED pixel strip.
  - A test unit for either remote testing independent of a DMX-512 bus or
    for onboard testing of electronics that are to be connected to a generic
    or pixel strip node.

Folders:
  Arduino_sketches:
    - Arduino sketches for gateway, generic and remote slave nodes, and
      test unit.

  Documentation:
    - User manuals, schematics, PCB files, construction guides.
