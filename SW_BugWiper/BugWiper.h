#include <sys/_stdint.h>
#include <Arduino.h>

#define PWM_FREQ 10000
#define PWM_RESOLUTION_BITS 8
#define MOTOR_CURRENT_STOP 2500
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
#define TIME_PWM_RAMP_START 120
#define TIME_PWN_RAMP_SLOW 80
#define TIME_PWM_RAMP_CLEANING 80     //time of PWM power inrements for start cleaning ramp
#define TIME_PWM_RAMP_WINDING_IN 20   //time of PWM power inrements for start winding in ramp
#define TIME_PWM_RAMP_LOOSE_CABLE 30  //time of PWM power inrements after loose cable ramp
#define TIME_MIN_CLEANING 300         //minimal cleaning time in ms
#define TIME_MAX_CLEANING 90000       //maximale cleaning time in ms
#define TIME_MAX_WINDING_IN 50000     //maximale winding in time in ms
#define TIME_BUTTON_DEBOUNCE 50       //time in ms for button debounce
#define POSITION_STARTING 200         // Slow start lenght in mm
#define POSITION_WINGTIP 6500         // End of the Wing in mm
#define LENGTH_SLOW 200               // Distance to slow down 

enum class direction { out,
                       in,
                       stop };

class BugWiper {
public:
  BugWiper(int LED_p, int m_adc, int m_in1, int m_in2, int m_pwm, int m_pwm_chnl);
  void init();
  void read_motor_current();
  void set_timer();
  void set_winding_in();
  void set_start_cleaning();
  void set_motor_brake();
  void state_machine();
  void calculate(int64_t count, bool button_cleaning, bool button_winding_in, bool sw_cable_loose);
  volatile uint16_t ADC_current_sense;
  volatile uint32_t timer_cleaning;
  uint16_t state_machine_state;
private:
  void set_motor_dir(direction dir);
  void set_motor_power();
  void LED_blinking();
  int motor_pwm_channel;
  gpio_num_t LED_pin;
  gpio_num_t motor_current_pin;
  gpio_num_t motor_pwm_pin;
  gpio_num_t motor_in1_pin;
  gpio_num_t motor_in2_pin;
  gpio_num_t motor_inh1_pin;
  gpio_num_t motor_inh2_pin;
  gpio_num_t motor_is1_pin;
  gpio_num_t motor_is2_pin;
  uint8_t motor_power;
  uint8_t motor_power_dest;
  volatile uint16_t timer_LED;
  uint16_t LED_time;
  uint8_t time_pwm_ramp;
  volatile uint8_t timer_motor_power;
  int64_t motor_count; // counts from encoder
  int32_t position; // position in mm converted from the encoder
  int16_t p_numerator = 1;
  int16_t p_denominator = 1;
  bool motor_inverted;
  bool cable_loose;
  direction moror_direction;
};
