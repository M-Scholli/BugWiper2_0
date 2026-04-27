#ifndef BUGWIPER_H
#define BUGWIPER_H

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

#define LED_TIME_CLEANING 500    //time blinking LED
#define LED_TIME_WINDING_IN 250  //time blinking LED

#define START_POWER_CLEANING 50     //start power cleaning
#define START_POWER_WINDING_IN 100   //start power winding in
#define START_POWER_LOOSE_CABLE 60  //start power after loose cable

#ifdef TESTBENCH
#define MAX_POWER_START_CLEANING 150  //max power at startion winding out
#define MAX_POWER_WINDING_OUT 200     //max power while cleaning
#define MAX_POWER_NEAR_END 30
#define MAX_POWER_WINDING_IN 255  //max power while winding in
#define MAX_POWER_GROUND 50       //max power on the ground
#else
#define MAX_POWER_START_CLEANING 150  //max power at startion winding out
#define MAX_POWER_WINDING_OUT 255     //max power while cleaning
#define MAX_POWER_NEAR_END 30
#define MAX_POWER_WINDING_IN 255  //max power while winding in
#define MAX_POWER_GROUND 50       //max power on the ground
#endif

#define TIME_MIN_CLEANING 300  //minimal cleaning time in ms

#define TIME_FINISH_RESET 1500
#define TIME_ERROR_RESET 3000
#define TIME_LONG_PRESS 200      //time in ms for long button press
#define TIME_BUTTON_DEBOUNCE 25  //time in ms for button debounce
#define TIME_MAX_DEBOUNCE 50
#define TIME_BUTTON_CLEAR 5

// Stop function
#ifdef TESTBENCH
#define BW_STOP_CURRENT 6500
#define BW_STOP_CURRENT_COUNTS 5
#define BW_STOP_SPEED 1
#define BW_STOP_SPEED_COUNTS 20
#define BW_STOP_V_BAT 8.0
#define BW_STOP_T_MAX 50
#define TIME_MAX_CLEANING 9000    //maximale cleaning time in ms
#define TIME_MAX_WINDING_IN 5000  //maximale winding in time in ms
#else
#define BW_STOP_CURRENT 7500
#define BW_STOP_CURRENT_COUNTS 5
#define BW_STOP_SPEED 1
#define BW_STOP_SPEED_COUNTS 20
#define BW_STOP_V_BAT 8.0
#define BW_STOP_T_MAX 70
#define TIME_MAX_CLEANING 90000    //maximale cleaning time in ms
#define TIME_MAX_WINDING_IN 50000  //maximale winding in time in ms
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
#define MOTOR_HB1_DK 28900
#define MOTOR_HB2_DK 28900

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
#define SPOOL_CIRCUMFERENCE 75.4  // in mm

#ifdef TESTBENCH
#define POSITION_STARTING 100  // Slow start lenght in mm
#define POSITION_SLOW_WINGTIP 1300
#define POSITION_WINGTIP 1600       // End of the Wing in mm
#define POSITION_SLOW_FUSELAGE 500  // End of the Wing in mm
#define LENGTH_SLOW 200             // Distance to slow down
#else
#define POSITION_STARTING 200  // Slow start lenght in mm
#define POSITION_SLOW_WINGTIP 6000
#define POSITION_WINGTIP 6500        // End of the Wing in mm
#define POSITION_SLOW_FUSELAGE 6500  // End of the Wing in mm
#define LENGTH_SLOW 200              // Distance to slow down
#endif

enum direction { OUT = 0,
                 IN,
                 STOP,
                 Freewheeling };

// Bug Wiper FSM operating modes
enum BW_MODE {
  M_IDLE = 0,             // Idle, motor off

  M_REFERENCE_IN,         // Reference move inward, zero encoder
  M_START_CLEAN_OUT,      // Slow start moving outward
  M_CLEANING,             // Normal cleaning movement

  M_DECEL_LOOSE,          // Decelerate after loose cable detected
  M_WIGGLE_LOOSE,         // Wiggle motor to release wiper
  M_RESTART_AFTER_LOOSE,  // Restart after successful wiggle

  M_DECEL_END,            // Decelerate before wingtip end
  M_WINDING_IN,           // Normal winding in

  M_GROUND_OUT,        // Ground mode: limited outward move only

  M_FINISHED,             // Cleaning finished

  M_EMERGENCY_IN,         // Emergency winding in (override)
  M_STOP,                 // Controlled stop
  M_ERROR,                // Error state

  BW_MODE_COUNT
};

// RGB COLOUR struct
struct RGB_COLOUR {
  uint8_t r, g, b;
};

// Configuration parameters defining behavior per FSM mode
struct ModeConfig {
  // Motor behavior
  direction dir;              // Initial motor direction on mode entry
  uint8_t   startPower;       // Motor power at mode entry
  uint8_t   maxPower;         // Maximum allowed motor power
  uint16_t  pwmRampTime;      // Time between PWM ramp steps [ms]

  // Time supervision
  uint32_t  minTime;          // Minimum time to stay in this mode [ms]
  uint32_t  maxTime;          // Maximum allowed time (0 = disabled) [ms]

  // LED indication
  RGB_COLOUR ledColor;        // LED color for this mode
  uint16_t   ledBlinkTime;    // LED blink period (0 = steady)

  // Behavior flags
  bool allowLooseDetect;      // Enable loose cable detection
  bool allowWiggle;           // Allow transition to wiggle mode
  bool ignoreSafety;          // Ignore safety checks (emergency)

  // FSM flow
  BW_MODE defaultNext;   // Nominal next state (BW_MODE_COUNT = none)
};


// User initiated operation context
enum UserCommand {
  CMD_NONE,
  CMD_CLEANING,
  CMD_WINDING_IN
};

enum Step {
  STEP_INIT,
  STEP_RUNNING
};

extern Step step;


struct PositionConfig {
  int32_t startSlowOut;
  int32_t slowZoneStart;
  int32_t wingTip;
  int32_t groundOutMax;
};

extern const PositionConfig positionConfig;


extern const ModeConfig modeConfig[BW_MODE_COUNT];

inline constexpr RGB_COLOUR BLACK = { 0, 0, 0 };
inline constexpr RGB_COLOUR RED = { 100, 0, 0 };
inline constexpr RGB_COLOUR GREEN = { 0, 100, 0 };
inline constexpr RGB_COLOUR BLUE = { 0, 0, 100 };
inline constexpr RGB_COLOUR YELLOW = { 100, 100, 0 };
inline constexpr RGB_COLOUR CYAN = { 0, 100, 100 };
inline constexpr RGB_COLOUR MAGENTA = { 100, 0, 100 };
inline constexpr RGB_COLOUR ORANGE = { 80, 35, 0 };

const char* bwModeToString(BW_MODE mode);
void BugWiper_rgbLed_init(void);
void BugWiper_init(void);
void BugWiper_rgbLedWrite(struct RGB_COLOUR colour);
void BugWiper_log(void);
void BugWiper_test_LED(void);
void BugWiper_test_Motor(void);
void BugWiper_read_motor_current(void);
void BugWiper_set_timer(void);
void BugWiper_set_winding_in(void);
void BugWiper_set_start_cleaning(void);
void BugWiper_state_machine(void);
void BugWiper_calculate(bool button_cleaning, bool button_winding_in, bool sw_cable_loose);

extern BW_MODE BugWiper_currentMode;
extern uint32_t modeStartTime;

extern ESP32Encoder BW_motor_encoder;
extern uint32_t BW_ADC_current_sense;
extern volatile double BW_ADC_current_mA_filtered;
extern float BW_ADC_T_ntc_degree;
extern float BW_ADC_V_Bat;
extern volatile double BW_ADC_btn_hb1;
extern volatile double BW_ADC_btn_hb2;

extern volatile uint32_t BW_state_machine_timer;
extern volatile int32_t BW_position;
extern volatile int32_t BW_speed;
extern volatile int64_t motor_enc_count;  // counts from encoder

#endif