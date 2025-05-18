#include <Arduino.h>
#include <ESP32Encoder.h>
#include "BugWiper.h"
#include "my_debug.h"

#define BugWiperPCB 1

#define ESP32_S3_DevKit 0

#if BugWiperPCB
#define BTN9960_CONTROLLER 1
#endif

#define FIRMWARE_VERSION "V0.0.2"
#define DEBUG_SERIAL_OUT 2

#define TRACE_LEVEL_INFO       4
#define TRACE_LEVEL_WARNING    3
#define TRACE_LEVEL_ERROR      2
#define TRACE_LEVEL_FATAL      1
#define TRACE_LEVEL_NO_TRACE   0

// Pin discription
// ESP32-Wroom-32: 
// follwing pins are not allowed to use: 0 (Bootselect); 2 Board LED / must be low on boot; (1 & 3 UART USB-Serial); 
// (5 must be high during boot);(15 Debugging Log on U0TXD During Booting);(6, 7, 8, 9, 10 & 11 connected to Flash);
// Input Only Pins : 34,35,36,39
// ESP32-S3-Wroom-1:
// follwing pins are difficult to use: 0 (Bootselect); 3 (Strapping Pins Floating) ; 19 & 20 (USB-Jtag); 35,36&37 (Octal PSRAM (8MB));
// 39,40,41&42 (JTAG); 43 & 44 (UART 0 for serial debug interface); 45 & 46 (Strapping Pins / Pull-down)  48 Board LED

//Button PINs
#if BugWiperPCB
  #define BUTTON_CLEANING_A_PIN 14
  #define BUTTON_WINDING_IN_A_PIN 21
  #define SW_CABLE_LOOSE_A_PIN 17
  #define SAFETY_SWITCH_PIN 47  // Saftyswitch to deaktivate the BugWiper
//LED configuration
  #define LED_PIN 9
//Motor PINs
  #define ADC_NTC_PIN 5
  #define ADC_VBat_PIN 4
  #define MOTOR_IN1_PIN 10
  #define MOTOR_IN2_PIN 11
  #define MOTOR_INH1_PIN 12
  #define MOTOR_INH2_PIN 13
  #define MOTOR_IS1_PIN 6
  #define MOTOR_IS2_PIN 7
  #define MOTOR_CURRENT_SENSE_PIN 1
  #define MOTOR_ENCODER_1_PIN 15
  #define MOTOR_ENCODER_2_PIN 16
//SD Card
  #define SD_Detect_PIN 2
#elif ESP32_S3_DevKit
  #define BUTTON_CLEANING_A_PIN 5
  #define BUTTON_WINDING_IN_A_PIN 6
  #define SW_CABLE_LOOSE_A_PIN 10
  #define SAFETY_SWITCH_PIN 11  // Saftyswitch to deaktivate the BugWiper
//LED configuration
  #define RGB_BUILD_IN 48
  #define LED_PIN 48
//Motor PINs
  #define MOTOR_IN1_PIN 12
  #define MOTOR_IN2_PIN 13
  #define MOTOR_EN_PIN 14  //PWM Pin
  #define MOTOR_CURRENT_SENSE_PIN 16
  #define MOTOR_ENCODER_1_PIN 1
  #define MOTOR_ENCODER_2_PIN 2
#else
  #define BUTTON_CLEANING_A_PIN 21
  #define BUTTON_WINDING_IN_A_PIN 18
  #define SW_CABLE_LOOSE_A_PIN 23
  #define SAFETY_SWITCH_PIN 16  // Saftyswitch to deaktivate the BugWiper
//LED configuration
  #define LED_PIN 2
//Motor PINs
  #define MOTOR_IN1_PIN 12
  #define MOTOR_IN2_PIN 13
  #define MOTOR_EN_PIN 14  //PWM Pin
  #define MOTOR_CURRENT_SENSE_PIN 36
  #define MOTOR_ENCODER_1_PIN 26
  #define MOTOR_ENCODER_2_PIN 27
#endif

// Kalibrierung des Putzvorganges
#define TIME_LONG_PRESS 400           //time in ms for long button press
#define MAX_POWER_CLEANING 255        //max power while cleaning
#define MAX_POWER_WINDING_IN 255      //max power while winding in
#define START_POWER_BRAKE 210         //power on start of the motorbrake
#define MAX_POWER_BRAKE 255           //max power of the motorbrake
#define LOOSE_POWER_BRAKE 240         //power of the brake when loose cable detected
#define TIME_PWM_RAMP_BRAKE 2         //time of PWM power inrements for braking ramp
#define TIME_PWM_RAMP_CLEANING 80     //time of PWM power inrements for start cleaning ramp
#define TIME_PWM_RAMP_WINDING_IN 20   //time of PWM power inrements for start winding in ramp
#define TIME_PWM_RAMP_LOOSE_CABLE 30  //time of PWM power inrements after loose cable ramp
#define TIME_MIN_CLEANING 300         //minimal cleaning time in ms
#define TIME_MAX_CLEANING 90000       //maximale cleaning time in ms
#define TIME_MAX_WINDING_IN 50000     //maximale winding in time in ms
#define START_POWER_CLEANING 30       //start power cleaning
#define START_POWER_WINDING_IN 70     //start power winding in
#define START_POWER_LOOSE_CABLE 60    //start power after loose cable
#define TIME_BUTTON_DEBOUNCE 50       //time in ms for button debounce

//PWM configuration
#define PWM_CHANNEL_A 0

//Global variables
uint16_t timer_button_long_press_a = 0;  // Time since start of long pressing button A
uint8_t timer_button_cable_loose_a = 0;
uint8_t timer_button_winding_in_a = 0;
uint8_t timer_button_start_cleaning_a = 0;
ESP32Encoder encoder_motor_a;


hw_timer_t *Timer0_Cfg = NULL;
volatile uint16_t counter_timer = 0;

#ifdef BTN9960_CONTROLLER
//BugWiper Putzi_a(LED_PIN, MOTOR_CURRENT_SENSE_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_EN_PIN, PWM_CHANNEL_A);
#else
//BugWiper Putzi_a(LED_PIN, MOTOR_CURRENT_SENSE_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_EN_PIN, PWM_CHANNEL_A);
#endif

void IRAM_ATTR Timer0_ISR(void) {
  counter_timer = counter_timer+1;
  BugWiper_read_motor_current();
  if (counter_timer == 10) {
    counter_timer = 0;
    BugWiper_set_timer();
  }
}

void Encoder_init(void) {
  encoder_motor_a.attachHalfQuad(MOTOR_ENCODER_1_PIN, MOTOR_ENCODER_2_PIN);
  encoder_motor_a.setCount(0);
}

void Timer_init(void) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Code for version 3.x
  Timer0_Cfg = timerBegin(1000000);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
  timerAlarm(Timer0_Cfg, 100, true, 0);
#else
  // Code for version 2.x
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100, true);
  timerAlarmEnable(Timer0_Cfg);
#endif
  
}

void init_io(void) {
  DEBUG_INFO("Init PINs A:")
  LED_pin = (gpio_num_t)LED_PIN;
  motor_current_pin = (gpio_num_t)MOTOR_CURRENT_SENSE_PIN;
  motor_in1_pin = (gpio_num_t)MOTOR_IN1_PIN;
  motor_in2_pin = (gpio_num_t)MOTOR_IN2_PIN;
  motor_inh1_pin = (gpio_num_t)MOTOR_INH1_PIN;
  motor_inh2_pin = (gpio_num_t)MOTOR_INH2_PIN;
  motor_is1_pin = (gpio_num_t)MOTOR_IS1_PIN;
  motor_is2_pin = (gpio_num_t)MOTOR_IS2_PIN;
  pinMode(SW_CABLE_LOOSE_A_PIN, INPUT_PULLUP);
  pinMode(SAFETY_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_WINDING_IN_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CLEANING_A_PIN, INPUT_PULLUP);
  DEBUG_INFO("Init PINs A complete")
}

void button_debounce(void) {
  if (digitalRead(SW_CABLE_LOOSE_A_PIN) == 0 && timer_button_cable_loose_a < 255) {
    timer_button_cable_loose_a++;
  } else if (digitalRead(SW_CABLE_LOOSE_A_PIN) && timer_button_cable_loose_a > 0) {
    timer_button_cable_loose_a--;
  }
  if (digitalRead(BUTTON_WINDING_IN_A_PIN) == 0 && timer_button_winding_in_a < 255) {
    timer_button_winding_in_a++;
  } else if (digitalRead(BUTTON_WINDING_IN_A_PIN) && timer_button_winding_in_a > 0) {
    timer_button_winding_in_a--;
  }
  if (digitalRead(BUTTON_CLEANING_A_PIN) == 0 && timer_button_start_cleaning_a < 255) {
    timer_button_start_cleaning_a++;
  } else if (digitalRead(BUTTON_CLEANING_A_PIN) && timer_button_start_cleaning_a > 0) {
    timer_button_start_cleaning_a--;
  }
}

void read_Buttons(void) {
  if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
    if (timer_button_winding_in_a >= TIME_BUTTON_DEBOUNCE && BW_state_machine_state == 0) {
      BugWiper_set_winding_in();
    }
    if (BW_state_machine_state == 0 && timer_button_long_press_a >= TIME_LONG_PRESS) {
      BugWiper_set_start_cleaning();
    }
  }
  // prevents a imediate second start cleaning after finish the first one
  if (timer_button_winding_in_a <= 5 && timer_button_start_cleaning_a <= 5
      && BW_state_machine_state == 3 && BW_timer_cleaning >= 200) {
    BW_state_machine_state = 0;
  }
  // reset timer for long press of buttons
  if (timer_button_start_cleaning_a <= 5) {
    timer_button_long_press_a = 0;
  }
}


//The setup function is called once at startup of the sketch
void setup() {
    DEBUG_INIT(115200);
    delay(500);
    DEBUG_INFO("BugWiper start programm");
    Encoder_init();
    init_io();
    BugWiper_init();
    Timer_init();
    if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
      DEBUG_INFO("SAFETY SWITCH closed");
    } else {
      DEBUG_WARNING("WARNING!!! SAFETY SWITCH open");
      DEBUG_INFO("Close the SAFETY SWITCH to operate the BugWiper");
    }
}

uint16_t counter_output=0;
// The loop function is called in an endless loop
void loop() { 
    read_Buttons();
    BugWiper_calculate(encoder_motor_a.getCount(), 0, 0, 0);
#if (DEBUG_SERIAL_OUT >= 2)
  if (counter_output > 10000) {
    DEBUG_INFO("ADC value = " + String(BW_ADC_current_sense));
    DEBUG_INFO("Encoder count = " + String((int32_t)encoder_motor_a.getCount()));
    counter_output = 0;
  }
  counter_output++;
#endif
}


