#include <Arduino.h>
#include "BugWiper.h"
#include "my_debug.h"



#define FIRMWARE_VERSION "V0.0.2"

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
    DEBUG_INFO("ADC Current = " + String(BW_ADC_current_mA) + " HB1:" + String(BW_ADC_btn_hb1)+" HB2:" + String(BW_ADC_btn_hb2));
    DEBUG_INFO("Encoder count = " + String((int32_t)motor_enc_count));
    DEBUG_INFO("Position = " + String(BW_position) + " Speed:" + String(BW_speed));
    DEBUG_INFO("State:" + String(BW_state_machine_state));
    float temp;
    uint16_t adc_temp;
    adc_temp = analogReadMilliVolts(ADC_NTC_PIN);
    temp = (float)adc_temp * 4095.0 / 3100.0;
    adc_temp = (uint16_t) temp;
    temp = ((float)adc_temp*(float)adc_temp*(float)adc_temp*(-2.87638e-9))+((float)adc_temp*(float)adc_temp*(2.01243e-5))+((-0.0702)*(float)adc_temp)+109.013;
    DEBUG_INFO("ADC VBat= " + String((float)((float)analogReadMilliVolts(ADC_VBat_PIN)*0.0081)) + " NTC:" + String((float)analogReadMilliVolts(ADC_NTC_PIN)) + " =" + String(temp));
    DEBUG_INFO("SW_C_loose= "+ String(cable_loose));
    DEBUG_INFO(" ");
}

