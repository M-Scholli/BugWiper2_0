# BugWiper

This project is an open source electronic bug wiper system. 

## Old Version
### Version 1
![Image](./fotos/old_version.jpg)

- Modelcraft RB350018-2A723R Hochleistungsgetriebemotor 12 V 18:1 :https://www.conrad.de/de/p/modelcraft-rb350018-2a723r-hochleistungsgetriebemotor-12-v-18-1-233131.html
- dual H-Bridge driver L298 3A


### Version 1.1
![Image](./fotos/old_version2.jpg)

- MFA 919D1481 Geared Motor 12 V/DC 148:1 https://amzn.eu/d/hulJdEZ
- Arduino Nano
- IBT_2 BTS7960B 43 A Motor Driver: https://www.amazon.de/dp/B09HGBM5D2

### Panel
 - one LED per Wing
 - one two-way toggle-switch
![Image](./fotos/panel.jpg)


## New ESP32 Version (Ideas):
### New mechanical design:
- ESP32 instead of 8bit Atmel controller 
-  no more end stop switch
-  no big spring for the end stop
- encoder for the cable spool
### New software design:
- motor current sensor and motor position decides when to stop the BugWiper motor
- programmable cable length with the motor encoder
- direction change of the motor at the wingtip. Motor turn every time the same direction for in and out.
- slow down the motor before reaching the fuselage to reduce the force at the stop.
- auto retighten the bug wipers if they loosen from the fuselage in flight
- ground modus to loosen the bug wiper just bit and no full cleaning process
- WiFi Hotspot with web browser to configure the BugWiper system before flight

## Used IDE and Settings:
- Arduino IDE2.3.2
- selected Board: ESP32S3 DEV Module

### Board Pinout:
![Image](./fotos/Pinout_ESP32-S3.png)
 
### Board Settings:
![Image](./fotos/settings.PNG)
 

### Installed Librarys:
- ESP32Encoder 0.11.6 by Kevin Harrington: https://github.com/madhephaestus/ESP32Encoder
- Manual Installed ESPAsyncWebServer by lacamera modified by Ed Nieuwenhuys to work with EPS32 V3: https://github.com/ednieuw/ESPAsyncWebServer
- Async TCP 3.1.4 by Me-No-Dev https://github.com/mathieucarbou/AsyncTCP


## Used Hardware:
- iHaospace 2 x ESP32-S3-DevKitC-1 N16R8 16Mb Flash, 8MB PSRAM https://www.amazon.de/dp/B0D1CBV999
- IBT_2 BTS7960B 43 A Motor Driver: https://www.amazon.de/dp/B09HGBM5D2
- 12V DC 200rpm High Torque Geared Electric Motor: https://www.amazon.de/gp/product/B00T48KC1Q 
- A3144 Linear Hall Effect Sensor: https://www.amazon.de/dp/B0BQ2Z335H
- Self-Adhesive Magnets 8 x 1 mm : https://www.amazon.de/dp/B0BJQ918KX
- Micro Switch / Flipper Coin Counter: https://www.amazon.de/dp/B00X5LQMMQ

- to make the cable drum:
CHANCS Aluminium Alloy Double V-Groove Belt Pulley 40 mm OD 8 mm Fixed Pulley for Motor Shaft Bore 6 mm: https://www.amazon.de/gp/product/B0CRR4KS3J

# Motor Tests:

## 5840-555 Worm Geared Motor
A Motor similar to https://nfpshop.com/product/12v-24v-metal-gear-worm-gear-model-nfp-5840-555-en.

### Motor Parameters:
- 12 Volt
- 30 Watt
- 470 rpm
- self locking worm gear
- 17:1 Gear ratio

### Testing

Cable drum with 24mm Diameter.

#### Forwards:
| Load| Current| Speed |
| :-- | :------: | ----: |
| 1kg |   2.2 A  | 0.55 m/s |
| 2kg |   3.4 A  | 0.44 m/s |
| 3kg |  4.6 A   | 0.39 m/s |
| 4kg |  6.2 A   | 0.32 m/s |
| 5kg |  8.3A    | 0.23 m/s |

#### Backwards:
| Load| Current| Speed |
| :-- | :------: | ----: |
| 2kg |  4.6 A  | 0.4 m/s |
| 3kg |  6.4 A  | 0.37 m/s |

### Result:
Positive:
+ self locking
+ high Torque
+ integrated Encoder
+ fast

Negative:
- high power consumption
- bad efficiency
- high noise
- higher losses when turning backwards

## ZGB37RH31
https://www.amazon.de/gp/product/B00T48KC1Q




## Open Source Software Declaration
The Webserver part is based on https://github.com/smford/esp32-asyncwebserver-fileupload-example and licensed under Apache 2.0 license