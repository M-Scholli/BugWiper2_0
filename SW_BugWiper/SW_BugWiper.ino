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
  BugWiper_test_LED();
}

// The loop function is called in an endless loop
void loop() {
  //read_Buttons();
  //BugWiper_calculate(0, 0, 0);
  sdLoggerHandleCard();
  unsigned long t = millis();
  sdLoggerLog(t, BW_state_machine_state, BW_position, BW_speed, BW_ADC_current_mA_filtered, BW_ADC_V_Bat);
  DEBUG_INFO("ADC_Current:" + String(BW_ADC_current_mA_filtered) + " HB1:" + String(BW_ADC_btn_hb1) + " HB2:" + String(BW_ADC_btn_hb2));
  DEBUG_INFO("Encoder_count:" + String((int32_t)motor_enc_count) + " Power:" + String(motor_power));
  DEBUG_INFO("Position:" + String(BW_position) + " Speed:" + String(BW_speed));
  DEBUG_INFO("State:" + String(BW_state_machine_state));
  DEBUG_INFO("ADC_VBat:" + String(BW_ADC_V_Bat) + " NTC:" + String(BW_ADC_T_ntc_degree));
  DEBUG_INFO("SW_loose:" + String(cable_loose));
  delay(500);
}
