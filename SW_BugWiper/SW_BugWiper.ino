#include <Arduino.h>
#include "BugWiper.h"
#include "my_debug.h"
#include "sd_logger.h"

#define FIRMWARE_VERSION "V0.0.3"

//The setup function is called once at startup of the sketch
void setup() {
  DEBUG_INIT(115200);
  BugWiper_rgbLed_init();
  delay(10);
  sdLoggerInit();
  if (sdLoggerAvailable()) {
    BugWiper_rgbLedWrite(GREEN);
  } else {
    BugWiper_rgbLedWrite(ORANGE);
  }
  delay(300);
  BugWiper_rgbLedWrite(BLACK);
  delay(200);
  DEBUG_INFO("BugWiper start programm");
  BugWiper_init();
  //Timer_init();
  if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
    DEBUG_INFO("SAFETY SWITCH closed");
  } else {
    DEBUG_WARNING("WARNING!!! SAFETY SWITCH open");
    DEBUG_INFO("Close the SAFETY SWITCH to operate the BugWiper");
  }
  //BugWiper_test_LED();
}

// The loop function is called in an endless loop
void loop() {
  //read_Buttons();
  //BugWiper_calculate(0, 0, 0);
  sdLoggerHandleCard();
  if(BW_state_machine_state!=0){
    BugWiper_log();
  }
  delay(500);
}
