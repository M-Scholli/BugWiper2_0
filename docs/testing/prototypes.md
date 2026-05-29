# First Prototype

## PCB fixes config and versions
|Name|Change|Reason|PCB1|PCB2|
|--|---|---|---|---|
|Jp1|3.3V |Encoder|x|x|
|Jp2|closed||x|?|
|Jp3|closed||x|?|
|Jp4|12V Panel||x|?|
|Jp5|closed| 5V -> 3.3V|x|?|
|Jp6|closed| 12v -> 3.3V|x|?|
|C36|10nF --> 100nF|Filter Taster|?|x|
|C37| 10nF --> 1nF|Filter Encoder|x|x|
|C38|10nF --> 1nF|Filter ENcoder|x|x|
|C4|100nF --> 1uF|Reset Filter|no|x|
|C4|100nF --> 470nF|Reset Filter|x|no|

## PCB tests
|Test|Result PCB1| Result PCB2|
|--|---|---|
|Wing Side|right side|left side|
|Thermal Interface|OK|OK|
|Voltage Regulator 12V-3.3V|OK|OK|
|Voltage Regulator 5V-3.3V|OK|OK|
|Standby Current in mA|10|13|
|USB-C Port|OK|OK|
|Motor driver|OK|OK|
|Motor Encoder|OK|OK|
|SD Card|OK|OK|
|ButtonIN|OK|OK|
|ButtonOUT|OK|OK|
|RBG LED|OK|OK|
|Cable Loose Sensor|OK|OK|
|Ground Sensor|canopy reed switch|canopy reed switch|
|Current Sensor|OK|OK|
|Voltage Sensor|OK|OK|
|Temperature Sensor|OK|OK|
|Current Sensor calibration|||
|Voltage Sensor calibration|||
|Motor Encoder calibration|||
|Ground Test after assembly| OK | OK |
|Flight Test| OK | OK |
