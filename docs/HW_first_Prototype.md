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
|Thermal Interface|OK|foil must be removed|
|Voltage Regulator 12V-3.3V|OK||
|Voltage Regulator 5V-3.3V|OK||
|USB-C Port|OK||
|Motor driver|OK||
|Motor Encoder|OK||
|SD Card|OK|OK|
|ButtonIN|OK||
|ButtonOUT|OK||
|RBG LED|OK||
|Cable Loose Sensor|||
|Ground Sensor|||
|Current Sensor|OK||
|Voltage Sensor|OK||
|Temperature Sensor|OK||
|Current Sensor calibration|||
|Voltage Sensor calibration|||
|Motor Encoder calibration|||
|Ground Test after assembly|||


