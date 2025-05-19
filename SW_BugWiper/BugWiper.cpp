#include "esp32-hal-ledc.h"
#include <sys/_stdint.h>
#include <Arduino.h>
#include "my_debug.h"
#include "BugWiper.h"
#include "btn99x0_motor_control.hpp"

volatile uint16_t BW_ADC_current_sense;

uint16_t BW_state_machine_state;

volatile uint32_t BW_timer_cleaning;
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
enum direction moror_direction;

using namespace btn99x0;

io_pins_t hb1_io_pins =
{
    MOTOR_IS1_PIN,
    MOTOR_IN1_PIN,
    MOTOR_INH1_PIN
};

io_pins_t hb2_io_pins =
{
    MOTOR_IS2_PIN,
    MOTOR_IN2_PIN,
    MOTOR_INH2_PIN
};

hw_conf_t hw_conf =
{
    1000, // Rsense in Ohm
    3.3,  // VOltage Range
    4095  // ADC Steps
};

DCShield shield(hb1_io_pins,hb2_io_pins,hw_conf);
MotorControl btn_motor_control(shield);
HalfBridge HalfBridge_1= shield.get_half_bridge(DCShield::HALF_BRIDGE_1);
HalfBridge HalfBridge_2 = shield.get_half_bridge(DCShield::HALF_BRIDGE_2);

void BugWiper_init(void) {
  DEBUG_INFO("Init BugWiper:");
  pinMode(LED_pin, OUTPUT);
  pinMode(motor_in1_pin, OUTPUT);
  pinMode(motor_in2_pin, OUTPUT);
  digitalWrite(LED_pin, 0);
  digitalWrite(motor_in1_pin, 1);
  digitalWrite(motor_in2_pin, 1);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Code for version 3.x
  ledcAttach(motor_pwm_pin, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcWrite(motor_pwm_pin, 0);
#else
  // Code for version 2.x
  ledcSetup(motor_pwm_channel, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcAttachPin(motor_pwm_pin, motor_pwm_channel);
  ledcWrite(motor_pwm_channel, 0);
#endif
}

// TEST Functions
void BugWiper_test_LED(void)
{
  DEBUG_INFO("Test LEDs")
  rgbLedWrite(LED_pin, RGB_BRIGHTNESS, 0, 0);  // Red
  delay(500);
  rgbLedWrite(LED_pin, 0, RGB_BRIGHTNESS, 0);  // Green
  delay(500);
  rgbLedWrite(LED_pin, 0, 0, RGB_BRIGHTNESS);  // Blue
  delay(500);
  rgbLedWrite(LED_pin, 0, 0, 0);  // Off / black
}

void BugWiper_test_Motor(void)
{
    DEBUG_INFO("Run forward for 1 sec...");
    btn_motor_control.set_speed(180);
    delay(1000);

    DEBUG_INFO("Load current (A): ");
    DEBUG_INFO(HalfBridge_1.get_load_current_in_amps());
    DEBUG_INFO(HalfBridge_2.get_load_current_in_amps());
    DEBUG_INFO(((float)analogReadMilliVolts(motor_current_pin)*0.005));
    delay(1000);

    DEBUG_INFO("Freewheel for 1 sec...");
    btn_motor_control.freewheel();
    delay(1000);

    DEBUG_INFO("Run backward for 1 sec...");
    btn_motor_control.set_speed(-180);
    delay(2000);

    DEBUG_INFO("Brake for 1 sec...");
    btn_motor_control.brake();
    delay(1000);
}

void BugWiper_read_motor_current(void) {
  BW_ADC_current_sense = analogRead(motor_current_pin);
}

void BugWiper_set_timer(void) {
  BW_timer_cleaning = BW_timer_cleaning + 1;
  timer_motor_power = timer_motor_power + 1;
  timer_LED = timer_LED + 1;
}

// switch the motor to out, in or stops
void BugWiper_set_motor_dir(enum direction dir) {
  switch (dir) {
    case OUT:  // out
      {
        digitalWrite(motor_in2_pin, 0);
        digitalWrite(motor_in1_pin, 1);
      }
      break;

    case IN:  // in
      {
        digitalWrite(motor_in1_pin, 0);
        digitalWrite(motor_in2_pin, 1);
      }
      break;

    case STOP:  // stop
      {
        digitalWrite(motor_in2_pin, 0);
        digitalWrite(motor_in1_pin, 0);
      }
      break;
  }
}

void BugWiper_set_motor_power(void) {
  if (timer_motor_power >= time_pwm_ramp) {
    if (motor_power < motor_power_dest) {
      motor_power++;
    } else if (motor_power > motor_power_dest) {
      motor_power--;
    }
    timer_motor_power = 0;
  }
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Code for version 3.x
  ledcWrite(motor_pwm_pin, motor_power);  
#else
  // Code for version 2.x
  ledcWrite(motor_pwm_channel, motor_power);
#endif
}

void BugWiper_LED_blinking(void) {
  if (timer_LED >= LED_time) {
    //digitalWrite(LED_pin, !digitalRead(LED_pin)); Fixme
    timer_LED = 0;
  }
}

void BugWiper_set_motor_brake(void) {
  timer_motor_power = 0;
  motor_power = START_POWER_BRAKE;
  time_pwm_ramp = TIME_PWM_RAMP_BRAKE;
  motor_power_dest = MAX_POWER_BRAKE;
  BugWiper_set_motor_dir(STOP);
  DEBUG_INFO("Start braking A");
}

void BugWiper_set_winding_in(void) {
  BW_state_machine_state = 2;
  timer_LED = 0;
  timer_motor_power = 0;
  BW_timer_cleaning = 0;
  motor_power = START_POWER_WINDING_IN;
  time_pwm_ramp = TIME_PWM_RAMP_WINDING_IN;
  motor_power_dest = MAX_POWER_WINDING_IN;
  BugWiper_set_motor_dir(OUT);
  DEBUG_INFO("Start winding in");
}

void BugWiper_set_start_cleaning(void) {
  BW_state_machine_state = 10;
  DEBUG_INFO("Start cleaning");
}

/* main_state machine 
 0 = ready
 1 = cleaning prozess
 2 = winding in prozess
 3 = retighten
 5 = finished
 6 = ERROR
 7 = Stopp
 */

/* main state machine
0X idling
0 = init idling

10X starting winding out

20X winding out

30X loose cable situation
0 =  loose cable: init /  detected
1 =  loose cable: stopping
2 = lose cable: stopped
3 = lose cable: restarting
4 = full speed reached

40x end of wing
0 = direction change init 
1 = direction change slow down
2 = direction change full stop
3 = direction cghange new direction inti
4 = direction change restart
5 = direction change full speed reached

50x winding in
0 = near fuselage position reached
1 = near fuselage slow down
2 = near fuselage slow speed reached
3 = near fuselage no movemend detected
4 = near fuselage stopping

60x near fuselage
0 = stopping init
1 = stopping start
2 = stopping full stop
3 = stopping wait
4 = stopping retighten
5 = stopping stop finished

70x stopping at fuselage

80x retighten
*/

void BugWiper_state_machine(void) {
  switch (BW_state_machine_state) {
    case 0:  // do nothing
      break;
    // 1x starting winding out
    case 10:
      timer_motor_power = 0;
      timer_LED = 0;
      BW_timer_cleaning = 0;
      motor_power = START_POWER_CLEANING;
      time_pwm_ramp = TIME_PWM_RAMP_START;
      motor_power_dest = MAX_POWER_START_CLEANING;
      BugWiper_set_motor_dir(OUT);
      BW_state_machine_state++;
      break;
    case 11:
      if (position > POSITION_STARTING) {
        BW_state_machine_state = 20;
      }
      if (cable_loose) {
        BW_state_machine_state = 30;
      }
      break;
    case 20:
      motor_power_dest = MAX_POWER_WINDING_OUT;
      time_pwm_ramp = TIME_PWM_RAMP_CLEANING;
      BW_state_machine_state++;
      break;
    case 21:
      if (position > (POSITION_WINGTIP - LENGTH_SLOW)){
        BW_state_machine_state = 40;
      }
      if (cable_loose) {
        BW_state_machine_state = 30;
      }
      break;
    case 30:
      BugWiper_set_motor_dir(STOP);
      motor_power = LOOSE_POWER_BRAKE;
      motor_power_dest = LOOSE_POWER_BRAKE;
      BW_state_machine_state++;
      DEBUG_INFO("Cable is loose");
      break;
    case 31:
      if (!cable_loose) {
        if (position < POSITION_STARTING) {
          BW_state_machine_state = 10;
        } else {
          motor_power = START_POWER_CLEANING;
          BugWiper_set_motor_dir(OUT);
          BW_state_machine_state = 20;
        }
      }
      break;
    case 40:
      motor_power_dest = MAX_POWER_NEAR_END;
      time_pwm_ramp = TIME_PWN_RAMP_SLOW;
      BW_state_machine_state++;
      break;
    case 41:
      if (position > POSITION_WINGTIP){
        BW_state_machine_state = 50;
      }
      break;
    case 50:
      timer_motor_power = 0;
      timer_LED = 0;
      motor_power = START_POWER_WINDING_IN;
      time_pwm_ramp = TIME_PWM_RAMP_WINDING_IN;
      motor_power_dest = MAX_POWER_WINDING_IN;
      BugWiper_set_motor_dir(IN);
      BW_state_machine_state++;
      break;
    case 51:
    if (position < LENGTH_SLOW){
        BW_state_machine_state = 60;
    }
    case 60:
      motor_power_dest = MAX_POWER_NEAR_END;
      time_pwm_ramp = TIME_PWN_RAMP_SLOW;
      BW_state_machine_state++;
      break;
    case 61:
    if(1){
        BW_state_machine_state = 70;
    }
    break;
    case 80:
    BugWiper_set_motor_brake();
    BW_state_machine_state++;
    break;
  }
}

void BugWiper_calculate(int64_t count, bool button_cleaning, bool button_winding_in, bool sw_cable_loose) {
  position = count * p_numerator / p_denominator;
  if (BW_state_machine_state < 10) {
    if (button_cleaning) {
      BW_state_machine_state = 10;
    }
    if (button_winding_in) {
      BW_state_machine_state = 50;
    }
  }
  //set_timer();
  BugWiper_state_machine();
  BugWiper_set_motor_power();
  BugWiper_LED_blinking();
}