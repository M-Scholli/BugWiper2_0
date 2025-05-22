#include <stdint.h>
#include <sys/_stdint.h>
#include <Arduino.h>
#include <ESP32Encoder.h>

#define BugWiperPCB 1

#if BugWiperPCB
#define BTN9960_CONTROLLER 1
#elif
#define BTS7960B_CONTROLLER 1
#endif

#define PWM_FREQ 10000
#define PWM_RESOLUTION_BITS 8
#define MOTOR_CURRENT_STOP 2300
//LED configuration
#define RGB_LED_PIN 9
#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)
#define COLOUR_RED {64,0,0}
#define COLOUR_GREEN {0,64,0}
#define COLOUR_BLUE {0,0,64}


#define LED_TIME_CLEANING 500    //time blinking LED
#define LED_TIME_WINDING_IN 250  //time blinking LED

#define START_POWER_CLEANING 30       //start power cleaning
#define START_POWER_WINDING_IN 70     //start power winding in
#define START_POWER_LOOSE_CABLE 60    //start power after loose cable
#define MAX_POWER_START_CLEANING 150  //max power at startion winding out
#define MAX_POWER_WINDING_OUT 255     //max power while cleaning
#define MAX_POWER_NEAR_END 150
#define MAX_POWER_WINDING_IN 255      //max power while winding in
#define MAX_POWER_GROUND 50          //max power on the ground
#define START_POWER_BRAKE 210         //power on start of the motorbrake
#define MAX_POWER_BRAKE 255           //max power of the motorbrake
#define LOOSE_POWER_BRAKE 240         //power of the brake when loose cable detected
#define TIME_PWM_RAMP_BRAKE 2         //time of PWM power inrements for braking ramp
#define TIME_PWM_RAMP_START 12
#define TIME_PWN_RAMP_SLOW 8
#define TIME_PWM_RAMP_CLEANING 8     //time of PWM power inrements for start cleaning ramp
#define TIME_PWM_RAMP_WINDING_IN 2   //time of PWM power inrements for start winding in ramp
#define TIME_PWM_RAMP_LOOSE_CABLE 3  //time of PWM power inrements after loose cable ramp

#define TIME_MIN_CLEANING 300         //minimal cleaning time in ms
#define TIME_MAX_CLEANING 90000       //maximale cleaning time in ms
#define TIME_MAX_WINDING_IN 50000     //maximale winding in time in ms
#define TIME_BUTTON_DEBOUNCE 50       //time in ms for button debounce

#define TIME_LONG_PRESS 400           //time in ms for long button press

#define TIME_BUTTON_DEBOUNCE 50       //time in ms for button debounce

#define LENGTH_SLOW 200               // Distance to slow down 

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
#define CURRENT_CAL_FACTOR 

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
#define POSITION_STARTING 200         // Slow start lenght in mm
#define POSITION_WINGTIP 6500         // End of the Wing in mm

enum direction { OUT = 0,
                 IN,
                 STOP,
                 Freewheeling };

enum BW_MODE { M_IDLE = 0,
               M_CLEANING,
               M_WINDING_IN,
               M_WAIT,
               M_STOP };

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

extern ESP32Encoder BW_motor_encoder;
extern volatile uint32_t BW_ADC_current_sense;
extern volatile uint32_t BW_timer_cleaning;
extern uint16_t BW_state_machine_state;
extern volatile int32_t BW_position;
extern volatile int64_t motor_enc_count; // counts from encoder
extern gpio_num_t LED_pin;
extern gpio_num_t motor_current_pin;
extern gpio_num_t motor_pwm_pin;
extern gpio_num_t motor_in1_pin;
extern gpio_num_t motor_in2_pin;