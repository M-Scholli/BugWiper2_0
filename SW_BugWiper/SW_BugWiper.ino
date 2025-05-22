#include <Arduino.h>
#include "BugWiper.h"
#include "my_debug.h"



#define FIRMWARE_VERSION "V0.0.2"

// Pin discription
// ESP32-Wroom-32: 
// follwing pins are not allowed to use: 0 (Bootselect); 2 Board LED / must be low on boot; (1 & 3 UART USB-Serial); 
// (5 must be high during boot);(15 Debugging Log on U0TXD During Booting);(6, 7, 8, 9, 10 & 11 connected to Flash);
// Input Only Pins : 34,35,36,39
// ESP32-S3-Wroom-1:
// follwing pins are difficult to use: 0 (Bootselect); 3 (Strapping Pins Floating) ; 19 & 20 (USB-Jtag); 35,36&37 (Octal PSRAM (8MB));
// 39,40,41&42 (JTAG); 43 & 44 (UART 0 for serial debug interface); 45 & 46 (Strapping Pins / Pull-down)  48 Board LED

//SD Card
  #define SD_Detect_PIN 2

//The setup function is called once at startup of the sketch
void setup() {
    DEBUG_INIT(115200);
    delay(500);
    DEBUG_INFO("BugWiper start programm");
    BugWiper_init();
    //Timer_init();
    if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
      DEBUG_INFO("SAFETY SWITCH closed");
    } else {
      DEBUG_WARNING("WARNING!!! SAFETY SWITCH open");
      DEBUG_INFO("Close the SAFETY SWITCH to operate the BugWiper");
    }
    BugWiper_test_LED();
}

// The loop function is called in an endless loop
void loop() { 
    //read_Buttons();
    //BugWiper_calculate(0, 0, 0);
    delay(1000);
    DEBUG_INFO("ADC value = " + String(BW_ADC_current_sense));
    DEBUG_INFO("Encoder count = " + String((int32_t)motor_enc_count));
    DEBUG_INFO("Position = " + String(BW_position));
    DEBUG_INFO("State:" + String(BW_state_machine_state));
}


