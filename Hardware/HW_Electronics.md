# Hardware

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

### Motor Driver Chip Suggestions to design a smaller PCB

#### IFX007TAUMA1 or BTN8962TAAUMA1
are similar to the BTS7960B that is no longer in production.

- https://www.digikey.it/de/products/detail/infineon-technologies/IFX007TAUMA1/9586755
- https://www.digikey.it/de/products/detail/infineon-technologies/BTN8962TAAUMA1/4772018

# PCB Design

## BOM

| Nr | Name | Manufacturer|Type | Link |
| :-- | :----------------: | :------: |:------: | :----: |
|1|ESP32-S3-WROOM-1-N16R8|Espressif|Controller|https://mou.sr/4j57iPN|
|1|UJ20-C-H-G-SMT-1-P16-TR |Same Sky|USB-C Connector|https://mou.sr/4adiw0M
|1| 1N5819HW-7-F | Diodes Incorporated|Schottky Diode 1A | https://mou.sr/3PtqzN7
|1|82400102|Würth|TVS Diode Array|
|2|EVP-AT2L1B000|Panasonic|switch|https://mou.sr/4j0OPno|



