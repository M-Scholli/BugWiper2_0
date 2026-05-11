#include <Arduino.h>
#include "BugWiper.h"
#include "my_debug.h"
#include "sd_logger.h"

#define FIRMWARE_VERSION "V0.0.4"

//The setup function is called once at startup of the sketch
void setup() {
  DEBUG_INIT(115200);
  bw_rgbLed_init();
  bw_init();
  sdLoggerInit();
  while(millis() < 600) {
    delay(100);
  }
  bw_rgbLedWrite(BLACK);
  sdLoggerOpenLogLater();
  while(millis() < 1200) {
    delay(100);
  }
  if (sdLoggerAvailable()) {
    bw_rgbLedWrite(GREEN);
  } else {
    bw_rgbLedWrite(ORANGE);
  }
  delay(300);
  bw_rgbLedWrite(BLACK);
  DEBUG_INFO("BugWiper start programm");
  bw_log();
}

// The loop function is called in an endless loop
void loop() {
  sdLoggerHandleCard();
  if(bw_currentMode!=M_IDLE){
    bw_log();
  }
  //bw_btn_log();
  delay(500);
}
