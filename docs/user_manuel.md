# User Manual

## 1. Overview

This system is designed for:
- Automatic cleaning of the glider wings during flight with bug wipers
- Easy installation and maintenance
- Data logging for performance analysis and optimization
- Future-proof design for additional features and improvements

Main components:
- Bug wiper motor system
- Panel PCB (user interface)
- Bug Wipers (not part of this project yet, but the system is designed to work with standard bug wipers)


## 2. Getting Started
### Modes
the system has two modes:
- Normal Mode: Full cleaning process with automatic return and wiggle mode if the bug wipers are stuck at the fuselage
- Ground Mode: Loosens the bug wipers just one meter without a full cleaning process for ground handling

## 3. Operation

### Controls (Panel PCB)
The Control Panel has a 3-Pos momentary toggle switch for user interaction:

##### Normal Mode:
- Position 1 → Start cleaning (winding out)
- Position 3 → Return (winding in)

##### Ground Mode:
- Position 1 → Start loosening (after 2 seconds long press to prevent accidental activation)
- Position 3 → Return (winding in)

##### Stopping the process:
Witch pressing the opposite direction of the toggle switch the process can be stopped.

##### Emergency Winding In Mode:
With holding the toggle switch at Return the precess enters an emergency winding in mode, with overwrites stop conditions and enables the motor to wind in with full power until the toggle switch is released. This can be used if a sensor or software has a malfunction and the system stops to early.

## 4. Indicators

### RGB LED Status Indicator:

#### System Startup
- System initializes automatically and shows the configuration with the RGB LED with two short blinks and then turns black to save power.
- First blink: shows the Mode (Green for normal, orange for ground mode)
- Second blink: shows the SD-card status (Green for OK, orange for no SD-card for logging)

#### Cleaning Process:

- Green blinking → Running
- Yellow blinking → Cable Loose Mode / Wiggle Out
- Blue blinking → Wiggle In / Winding In
- Green long → Finished
- Red long → Error
