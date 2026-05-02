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

// Pin discription
// ESP32-Wroom-32:
// following pins are not allowed to use: 0 (Bootselect); 2 Board LED / must be low on boot; (1 & 3 UART USB-Serial);
// (5 must be high during boot);(15 Debugging Log on U0TXD During Booting);(6, 7, 8, 9, 10 & 11 connected to Flash);
// Input Only Pins : 34,35,36,39
// ESP32-S3-Wroom-1:
// following pins are difficult to use: 0 (Bootselect); 3 (Strapping Pins Floating) ; 19 & 20 (USB-Jtag); 35,36&37 (Octal PSRAM (8MB));
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
#define BTN_OUT_PIN 21
#define BTN_IN_PIN 14
#define SW_CABLE_LOOSE_PIN 18
#define SW_GROUND_PIN 47  // Safety switch to deactivate the BugWiper

// Invert ground switch logic
// #define BW_GROUND_SWITCH_INVERTED

// Invert cable loose switch logic
// #define BW_CABLE_LOOSE_INVERTED

// Encoder
#define MOTOR_ENCODER_1_PIN 16
#define MOTOR_ENCODER_2_PIN 17
#define CPR_Encoder 32
#define GEAR_RATIO 18
#define SPOOL_CIRCUMFERENCE 75.4  // in mm

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

enum direction {
  OUT = 0,
  IN,
  STOP,
  Freewheeling
};

typedef enum {
  BTN_IDLE,
  BTN_DEBOUNCE,
  BTN_PRESSED,
  BTN_HELD
} ButtonState;

typedef enum {
  BTN_EVT_NONE,
  BTN_EVT_SHORT,   // short press released
  BTN_EVT_LONG,    // long press detected
  BTN_EVT_RELEASE  // released after long press
} ButtonEvent;

struct MotorCommand {
  direction dir;
  uint8_t   startPower;
  uint8_t   targetPower;
  uint8_t   rampTime;
};

struct MotorState {
  // Motor control fields
  direction dir;
  uint8_t   power;
  uint8_t   targetPower;
  uint8_t   rampTime;
  uint8_t   rampTimer;
  
  // Motor state fields
  volatile int64_t encoder_count;
  volatile int32_t position_mm;
  volatile int32_t speed;
};

// RGB COLOUR struct
struct RGB_COLOUR {
  uint8_t r, g, b;
};

// LED behavior per FSM mode
struct LedCommand {
  RGB_COLOUR color;     // LED color
  uint16_t   blinkTime; // 0 = steady, >0 = blink period (ms)
};

// LED runtime state
struct LedState {
  RGB_COLOUR color;     // active color
  uint16_t   blinkTicks;// blink period in task ticks
  uint16_t   timer;     // tick counter
  bool       isOn;      // output state
};

// Configuration parameters defining behavior per FSM mode
typedef struct {
  MotorCommand motorCmd; // Motor intent for this mode
  LedCommand   ledCmd;   // LED intent

  // Time supervision
  uint32_t  minTime;          // Minimum time to stay in this mode [ms]
  uint32_t  maxTime;          // Maximum allowed time (0 = disabled) [ms]

  // Global transition flags
  bool enableEndCheckCurrent; // Enable current-based end position check in global transitions
  bool enableEndCheckSpeed;   // Enable speed-based end position check in global transitions
  bool enableSafetyProtection; // Enable safety protection check in global transitions
  bool enableMaxTimeCheck;    // Enable max time check in global transitions
  bool requireMinTimeForEndCheck; // Require minTime to elapse before any endCheck can trigger

  // FSM flow
  BW_MODE defaultNext;   // Nominal next state (BW_MODE_COUNT = none)
} BW_ModeConfig;

// User initiated operation context
enum UserCommand {
  CMD_NONE,
  CMD_CLEANING,
  CMD_WINDING_IN
};

typedef enum {
  SUB_INIT,
  SUB_RUNNING,
  SUB_DONE
} BW_SubState;

struct PositionConfig {
  int32_t startSlowOut;
  int32_t slowZoneStart;
  int32_t wingTip;
  int32_t groundOutMax;
};

struct ADCFilter {
  uint32_t filter_sum;
  uint32_t old_values[32];  // ADC_FILTER_SIZE
  int8_t counter;
};

struct ADCFiltered {
  volatile double current_mA_filtered;
  float temperature_degree;
  float battery_voltage;
  // HB1/HB2 omitted as requested
};

struct StopDetection {
  uint16_t current_counter;
  uint16_t speed_counter;
};

// Generic digital input filter
typedef struct {
  uint16_t count;      // integrator counter
  uint16_t threshold;  // activation threshold
  bool     state;      // filtered state
} BwFilter;

typedef struct {
  ButtonState state;    // internal FSM
  ButtonEvent event;    // one-shot event

  uint16_t debounceCnt; // debounce counter
  uint16_t holdCnt;     // hold counter

  bool rawLevel;        // raw GPIO level
} ButtonRuntime;

extern const BW_ModeConfig bw_modeConfig[BW_MODE_COUNT];

inline constexpr RGB_COLOUR BLACK = { 0, 0, 0 };
inline constexpr RGB_COLOUR RED = { 100, 0, 0 };
inline constexpr RGB_COLOUR GREEN = { 0, 100, 0 };
inline constexpr RGB_COLOUR BLUE = { 0, 0, 100 };
inline constexpr RGB_COLOUR YELLOW = { 100, 100, 0 };
inline constexpr RGB_COLOUR CYAN = { 0, 100, 100 };
inline constexpr RGB_COLOUR MAGENTA = { 100, 0, 100 };
inline constexpr RGB_COLOUR ORANGE = { 80, 35, 0 };

const char* bw_ModeToString(BW_MODE mode);
void bw_rgbLed_init(void);
void bw_init(void);
void bw_rgbLedWrite(struct RGB_COLOUR colour);
void bw_log(void);

extern BW_MODE bw_currentMode;

extern ESP32Encoder bw_motorEncoder;

// Neue strukturierte ADC-Variablen
extern ADCFilter adc_filter;
extern ADCFiltered adc_filtered;
extern StopDetection stop_detection;

#endif