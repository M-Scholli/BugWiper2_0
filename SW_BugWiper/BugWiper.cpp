#include "esp32-hal-ledc.h"
#include <sys/_stdint.h>
#include <Arduino.h>
#include "BugWiper.h"

BugWiper::BugWiper(int LED_p, int m_adc, int m_in1, int m_in2, int m_pwm, int m_pwm_chnl)
  : motor_pwm_channel{m_pwm_chnl},
    LED_pin{ (gpio_num_t)LED_p },
    motor_current_pin{ (gpio_num_t)m_adc },
    motor_pwm_pin{ (gpio_num_t)m_pwm },
    motor_in1_pin{ (gpio_num_t)m_in1 },
    motor_in2_pin{ (gpio_num_t)m_in2 } {}

void BugWiper::init() {
#if (DEBUG_SERIAL_OUT)
  Serial.println("Init BugWiperA:");
#endif
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

void BugWiper::read_motor_current() {
  ADC_current_sense = analogRead(motor_current_pin);
}

void BugWiper::set_timer() {
  timer_cleaning = timer_cleaning + 1;
  timer_motor_power = timer_motor_power + 1;
  timer_LED = timer_LED + 1;
}

// switch the motor to out, in or stops
void BugWiper::set_motor_dir(direction dir) {
  switch (dir) {
    case direction::out:  // out
      {
        digitalWrite(motor_in2_pin, 0);
        digitalWrite(motor_in1_pin, 1);
      }
      break;

    case direction::in:  // in
      {
        digitalWrite(motor_in1_pin, 0);
        digitalWrite(motor_in2_pin, 1);
      }
      break;

    case direction::stop:  // stop
      {
        digitalWrite(motor_in2_pin, 0);
        digitalWrite(motor_in1_pin, 0);
      }
      break;
  }
}

void BugWiper::set_motor_power() {
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

void BugWiper::LED_blinking() {
  if (timer_LED >= LED_time) {
    digitalWrite(LED_pin, !digitalRead(LED_pin));
    timer_LED = 0;
  }
}

void BugWiper::set_motor_brake() {
  timer_motor_power = 0;
  motor_power = START_POWER_BRAKE;
  time_pwm_ramp = TIME_PWM_RAMP_BRAKE;
  motor_power_dest = MAX_POWER_BRAKE;
  set_motor_dir(direction::stop);
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start braking A");
#endif
}

void BugWiper::set_winding_in(void) {
  state_machine_state = 2;
  timer_LED = 0;
  timer_motor_power = 0;
  timer_cleaning = 0;
  motor_power = START_POWER_WINDING_IN;
  time_pwm_ramp = TIME_PWM_RAMP_WINDING_IN;
  motor_power_dest = MAX_POWER_WINDING_IN;
  set_motor_dir(direction::out);
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start winding in A");
#endif
}

void BugWiper::set_start_cleaning(void) {
  state_machine_state = 10;
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start cleaning A");
#endif
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

void BugWiper::state_machine(void) {
  switch (state_machine_state) {
    case 0:  // do nothing
      break;
    // 1x starting winding out
    case 10:
      timer_motor_power = 0;
      timer_LED = 0;
      timer_cleaning = 0;
      motor_power = START_POWER_CLEANING;
      time_pwm_ramp = TIME_PWM_RAMP_START;
      motor_power_dest = MAX_POWER_START_CLEANING;
      set_motor_dir(direction::out);
      state_machine_state++;
      break;
    case 11:
      if (position > POSITION_STARTING) {
        state_machine_state = 20;
      }
      if (cable_loose) {
        state_machine_state = 30;
      }
      break;
    case 20:
      motor_power_dest = MAX_POWER_WINDING_OUT;
      time_pwm_ramp = TIME_PWM_RAMP_CLEANING;
      state_machine_state++;
      break;
    case 21:
      if (position > (POSITION_WINGTIP - LENGTH_SLOW)){
        state_machine_state = 40;
      }
      if (cable_loose) {
        state_machine_state = 30;
      }
      break;
    case 30:
      set_motor_dir(direction::stop);
      motor_power = LOOSE_POWER_BRAKE;
      motor_power_dest = LOOSE_POWER_BRAKE;
      state_machine_state++;
#if (DEBUG_SERIAL_OUT)
      Serial.println("Cable is loose");
#endif
      break;
    case 31:
      if (!cable_loose) {
        if (position < POSITION_STARTING) {
          state_machine_state = 10;
        } else {
          motor_power = START_POWER_CLEANING;
          set_motor_dir(direction::out);
          state_machine_state = 20;
        }
      }
      break;
    case 40:
      motor_power_dest = MAX_POWER_NEAR_END;
      time_pwm_ramp = TIME_PWN_RAMP_SLOW;
      state_machine_state++;
      break;
    case 41:
      if (position > POSITION_WINGTIP){
        state_machine_state = 50;
      }
      break;
    case 50:
      timer_motor_power = 0;
      timer_LED = 0;
      motor_power = START_POWER_WINDING_IN;
      time_pwm_ramp = TIME_PWM_RAMP_WINDING_IN;
      motor_power_dest = MAX_POWER_WINDING_IN;
      set_motor_dir(direction::in);
      state_machine_state++;
      break;
    case 51:
    if (position < LENGTH_SLOW){
        state_machine_state = 60;
    }
    case 60:
      motor_power_dest = MAX_POWER_NEAR_END;
      time_pwm_ramp = TIME_PWN_RAMP_SLOW;
      state_machine_state++;
      break;
    case 61:
    if(1){
        state_machine_state = 70;
    }
    break;
    case 80:
    set_motor_brake();
    state_machine_state++;
    break;
  }
}

void BugWiper::calculate(int64_t count, bool button_cleaning, bool button_winding_in, bool sw_cable_loose) {
  position = count * p_numerator / p_denominator;
  if (state_machine_state < 10) {
    if (button_cleaning) {
      state_machine_state = 10;
    }
    if (button_winding_in) {
      state_machine_state = 50;
    }
  }
  //set_timer();
  state_machine();
  set_motor_power();
  LED_blinking();
}