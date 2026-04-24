# Suggestions for alternative hardware selection

## Controller
I suggest to use a ESP32-WROOM-32 or ESP32-S3-WROOM.

## ESP32-WROOM-32
Simple and small solution:
- diymore ESP32 WROOM 32 Nodemcu https://amzn.eu/d/j1bOF2C
### Pinout SP32-S3-DevKitC-1 N16R8:
![Image](../fotos/Pinout_ESP32.jpg)

### CH340 Driver (USB-Serial)
https://www.arduined.eu/ch340-windows-10-driver-download/
### Board Settings:
![Image](../fotos/settings_ESP32.PNG)

## ESP32-S3-WROOM
newer ESP-S3:
- iHaospace 2 x ESP32-S3-DevKitC-1 N16R8 16Mb Flash, 8MB PSRAM https://www.amazon.de/dp/B0D1CBV999
### Pinout SP32-S3-DevKitC-1 N16R8:
![Image](../fotos/Pinout_ESP32-S3.png)
### Board Settings:
![Image](../fotos/settings_ESP32-S3.PNG)

## Motor controller / H-Bridge
### Complete Motor Driver PCB
#### IBT_2 BTS7960B 43 A Motor Driver: 
- https://www.makershop.de/module/motosteuerung/double-bts7960/
- https://www.amazon.de/dp/B09HGBM5D2

![image](../fotos/Pinout_BTS7960B.png)

Specs:
- Max Current: 43 A
- Voltage Range: 5 V - 27 V
- Max PWM freq.: 25 kHz
- Current sensing output

![image](../fotos/schematics_BTS7960B.png)

### Others
- IBT_2 BTS7960B 43 A Motor Driver: https://www.amazon.de/dp/B09HGBM5D2
- 12V DC 200rpm High Torque Geared Electric Motor: https://www.amazon.de/gp/product/B00T48KC1Q 
- A3144 Linear Hall Effect Sensor: https://www.amazon.de/dp/B0BQ2Z335H
- Self-Adhesive Magnets 8 x 1 mm : https://www.amazon.de/dp/B0BJQ918KX