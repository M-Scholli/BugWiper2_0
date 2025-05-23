#include <stdint.h>
#include <sys/_stdint.h>
#include <Arduino.h>
#include <ESP32Encoder.h>

#define TESTBENCH 1

#define BugWiperPCB 1

#if BugWiperPCB
#define BTN9960_CONTROLLER 1
#elif
#define BTS7960B_CONTROLLER 1
#endif

#define PWM_FREQ 10000
#define PWM_RESOLUTION_BITS 8

//LED configuration
#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)
#define COLOUR_RED {64,0,0}
#define COLOUR_GREEN {0,64,0}
#define COLOUR_BLUE {0,0,64}
#define LED_TIME_CLEANING 500    //time blinking LED
#define LED_TIME_WINDING_IN 250  //time blinking LED

#define START_POWER_CLEANING 50       //start power cleaning
#define START_POWER_WINDING_IN 60     //start power winding in
#define START_POWER_LOOSE_CABLE 60    //start power after loose cable
#ifdef TESTBENCH
  #define MAX_POWER_START_CLEANING 100  //max power at startion winding out
  #define MAX_POWER_WINDING_OUT 200     //max power while cleaning
  #define MAX_POWER_NEAR_END 100
  #define MAX_POWER_WINDING_IN 255      //max power while winding in
  #define MAX_POWER_GROUND 50          //max power on the ground
#else
  #define MAX_POWER_START_CLEANING 150  //max power at startion winding out
  #define MAX_POWER_WINDING_OUT 255     //max power while cleaning
  #define MAX_POWER_NEAR_END 150
  #define MAX_POWER_WINDING_IN 255      //max power while winding in
  #define MAX_POWER_GROUND 50          //max power on the ground
#endif
#define START_POWER_BRAKE 210         //power on start of the motorbrake
#define MAX_POWER_BRAKE 255           //max power of the motorbrake
#define LOOSE_POWER_BRAKE 240         //power of the brake when loose cable detected
#define TIME_PWM_RAMP_BRAKE 2         //time of PWM power inrements for braking ramp
#define TIME_PWM_RAMP_START 8
#define TIME_PWN_RAMP_SLOW 8
#define TIME_PWM_RAMP_CLEANING 6     //time of PWM power inrements for start cleaning ramp
#define TIME_PWM_RAMP_WINDING_IN 4   //time of PWM power inrements for start winding in ramp
#define TIME_PWM_RAMP_LOOSE_CABLE 4  //time of PWM power inrements after loose cable ramp

#define TIME_MIN_CLEANING 300         //minimal cleaning time in ms

#define TIME_FINISH_RESET 1500
#define TIME_ERROR_RESET 3000
#define TIME_LONG_PRESS 200           //time in ms for long button press
#define TIME_BUTTON_DEBOUNCE 25       //time in ms for button debounce
#define TIME_MAX_DEBOUNCE 50
#define TIME_BUTTON_CLEAR 5

// Stop function
#ifdef TESTBENCH
  #define BW_STOP_CURRENT 6500
  #define BW_STOP_CURRENT_COUNTS 5
  #define BW_STOP_SPEED 1
  #define BW_STOP_SPEED_COUNTS 20
  #define BW_STOP_V_BAT 9.0
  #define BW_STOP_T_MAX 50
  #define TIME_MAX_CLEANING 9000       //maximale cleaning time in ms
  #define TIME_MAX_WINDING_IN 5000     //maximale winding in time in ms
#else
  #define BW_STOP_CURRENT 7500
  #define BW_STOP_CURRENT_COUNTS 5
  #define BW_STOP_SPEED 1
  #define BW_STOP_SPEED_COUNTS 20
  #define BW_STOP_V_BAT 8.0
  #define BW_STOP_T_MAX 70
  #define TIME_MAX_CLEANING 90000       //maximale cleaning time in ms
  #define TIME_MAX_WINDING_IN 50000     //maximale winding in time in ms
#endif

// Pin discription
// ESP32-Wroom-32: 
// follwing pins are not allowed to use: 0 (Bootselect); 2 Board LED / must be low on boot; (1 & 3 UART USB-Serial); 
// (5 must be high during boot);(15 Debugging Log on U0TXD During Booting);(6, 7, 8, 9, 10 & 11 connected to Flash);
// Input Only Pins : 34,35,36,39
// ESP32-S3-Wroom-1:
// follwing pins are difficult to use: 0 (Bootselect); 3 (Strapping Pins Floating) ; 19 & 20 (USB-Jtag); 35,36&37 (Octal PSRAM (8MB));
// 39,40,41&42 (JTAG); 43 & 44 (UART 0 for serial debug interface); 45 & 46 (Strapping Pins / Pull-down)  48 Board LED
#define RGB_LED_PIN 9

//Motor PINs
#define MOTOR_IN1_PIN 10
#define MOTOR_IN2_PIN 11
#define MOTOR_INH1_PIN 12
#define MOTOR_INH2_PIN 13
#define MOTOR_IS1_PIN 6
#define MOTOR_IS2_PIN 7
#define MOTOR_HB1_DK  28900
#define MOTOR_HB2_DK  28900

#define MOTOR_CURRENT_SENSE_PIN 1
#define CURRENT_CAL_FACTOR 5.0

#define ADC_NTC_PIN 5
#define ADC_VBat_PIN 4

//PWM configuration
#define PWM_CHANNEL_A 0

//Button PINs
#define BUTTON_CLEANING_PIN 21
#define BUTTON_WINDING_IN_PIN 14
#define SW_CABLE_LOOSE_PIN 18
#define SAFETY_SWITCH_PIN 47  // Saftyswitch to deaktivate the BugWiper

// Encoder 
#define MOTOR_ENCODER_1_PIN 16
#define MOTOR_ENCODER_2_PIN 17
#define CPR_Encoder 32
#define GEAR_RATIO 18
#define SPOOL_CIRCUMFERENCE 75.4 // in mm

#ifdef TESTBENCH
  #define POSITION_STARTING 100         // Slow start lenght in mm
  #define POSITION_SLOW_WINGTIP 700
  #define POSITION_WINGTIP 1000         // End of the Wing in mm
  #define POSITION_SLOW_FUSELAGE 500   // End of the Wing in mm
  #define LENGTH_SLOW 200               // Distance to slow down 
#else
  #define POSITION_STARTING 200         // Slow start lenght in mm
  #define POSITION_SLOW_WINGTIP 6000
  #define POSITION_WINGTIP 6500         // End of the Wing in mm
  #define POSITION_SLOW_FUSELAGE 6500         // End of the Wing in mm
  #define LENGTH_SLOW 200               // Distance to slow down 
#endif

// STATE MACHINE STATE
#define BW_STATE_IDLE 0
#define BW_STATE_START_CLEANING 10
#define BW_STATE_START_WINDING_IN 60
#define BW_STATE_CHECK_END 61
#define BW_STATE_FINISHED 80
#define BW_STATE_STOP 100
#define BW_STATE_ERROR 100

enum direction { OUT = 0,
                 IN,
                 STOP,
                 Freewheeling };

enum BW_MODE { M_IDLE = 0,
               M_CLEANING,
               M_WINDING_IN,
               M_WAIT,
               M_FINISHED,
               M_STOP,
               M_ERROR };

struct RBG_COLOUR {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

void BugWiper_init(void);
void BugWiper_test_LED(void);
void BugWiper_test_Motor(void);
void BugWiper_read_motor_current(void);
void BugWiper_set_timer(void);
void BugWiper_set_winding_in(void);
void BugWiper_set_start_cleaning(void);
void BugWiper_set_motor_brake(void);
void BugWiper_state_machine(void);
void BugWiper_calculate(bool button_cleaning, bool button_winding_in, bool sw_cable_loose);

static const struct RBG_COLOUR ModeLED_Colour[]={
  {0,0,0},
  {0,100,30},
  {10,0,100},
  {100,30,5},
  {0,150,0},
  {100,50,15},
  {100,0,0}
};

extern ESP32Encoder BW_motor_encoder;
extern uint32_t BW_ADC_current_sense;
extern volatile double BW_ADC_current_mA_filtered;
extern float BW_ADC_T_ntc_degree;
extern float BW_ADC_V_Bat;
extern volatile double BW_ADC_btn_hb1;
extern volatile double BW_ADC_btn_hb2;

extern volatile uint32_t BW_state_machine_timer;
extern uint16_t BW_state_machine_state;
extern volatile int32_t BW_position;
extern volatile int32_t BW_speed;
extern volatile int64_t motor_enc_count; // counts from encoder

extern uint8_t motor_power;
extern bool cable_loose;
