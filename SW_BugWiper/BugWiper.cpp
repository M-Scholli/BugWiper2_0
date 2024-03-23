#include <Arduino.h>
#include "BugWiper.h"

BugWiper::BugWiper(int LED_p, int m_adc, int m_in1, int m_in2, int m_pwm, int m_pwm_chnl)
  : motor_pwm_channel{ m_pwm_chnl },
    LED_pin{ (gpio_num_t)LED_p },
    motor_current_pin{ (gpio_num_t)m_adc },
    motor_pwm_pin{ (gpio_num_t)m_pwm },
    motor_in1_pin{ (gpio_num_t)m_in1 },
    motor_in2_pin{ (gpio_num_t)m_in2 } {}

void BugWiper::init() {
  pinMode(LED_pin, OUTPUT);
  pinMode(motor_in1_pin, OUTPUT);
  pinMode(motor_in2_pin, OUTPUT);
  digitalWrite(LED_pin, 0);
  digitalWrite(motor_in1_pin, 1);
  digitalWrite(motor_in2_pin, 1);
  ledcSetup(motor_pwm_channel, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcAttachPin(motor_pwm_pin, motor_pwm_channel);
  ledcWrite(motor_pwm_channel, 0);
}

void BugWiper::read_motor_current() {
  ADC_current_sense = analogRead(motor_current_pin);
}

void BugWiper::set_timer() {
  timer_cleaning++;
  timer_motor_power++;
  timer_LED++;
}

// switch the motor 1 = out;	2 = in 2;	3 = stop;
void BugWiper::set_motor_dir(uint8_t dir) {
  switch (dir) {
    case 1:  // out
      {
        digitalWrite(motor_in2_pin, 0);
        digitalWrite(motor_in1_pin, 1);
      }
      break;

    case 2:  // in
      {
        digitalWrite(motor_in1_pin, 0);
        digitalWrite(motor_in2_pin, 1);
      }
      break;

    case 3:  // stop
      {
        digitalWrite(motor_in2_pin, 0);
        digitalWrite(motor_in1_pin, 0);
      }
      break;
  }
}

void BugWiper::set_motor_power() {
  if (timer_motor_power >= time_pwm_ramp) {
    if (motor_power < motor_power_max) {
      motor_power++;
    }
    timer_motor_power = 0;
  }
  ledcWrite(motor_pwm_channel, motor_power);
}

void BugWiper::set_motor_brake() {
  timer_motor_power = 0;
  motor_power = START_POWER_BRAKE;
  time_pwm_ramp = TIME_PWM_RAMP_BRAKE;
  motor_power_max = MAX_POWER_BRAKE;
  set_motor_dir(3);
  state_machine_main_state = 7;
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start braking A");
#endif
}

void BugWiper::set_winding_in(void) {
  state_machine_main_state = 2;
  timer_LED = 0;
  timer_motor_power = 0;
  timer_cleaning = 0;
  motor_power = START_POWER_WINDING_IN;
  time_pwm_ramp = TIME_PWM_RAMP_WINDING_IN;
  motor_power_max = MAX_POWER_WINDING_IN;
  if (motor_inverted) {
    set_motor_dir(1);
  } else {
    set_motor_dir(2);
  }
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start winding in A");
#endif
}

void BugWiper::set_start_cleaning(void) {
  state_machine_main_state = 1;
  timer_motor_power = 0;
  timer_LED = 0;
  timer_cleaning = 0;
  motor_power = START_POWER_CLEANING;
  time_pwm_ramp = TIME_PWM_RAMP_CLEANING;
  motor_power_max = MAX_POWER_CLEANING;
  if (motor_inverted) {
    set_motor_dir(2);
  } else {
    set_motor_dir(1);
  }
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start cleaning A");
#endif
}

void BugWiper::state_machine(bool button_cleaning, bool button_winding_in, bool sw_cable_loose) {
  switch (state_machine_main_state) {
    case 0:  // do nothing
      break;
    case 1:  // cleaning
      set_motor_power();
      // override function when start cleaning is hold pressed
      if (!button_cleaning) {
        // safty time out
        if (timer_cleaning >= TIME_MAX_CLEANING) {
          state_machine_main_state = 6;
        }
        // stop with pressing the other button:
        if (button_winding_in) {
          state_machine_main_state = 6;
        }
        // stop at high motor currents
        if (ADC_current_sense >= MOTOR_CURRENT_STOP) {
          state_machine_main_state = 5;
        }
        // check cable loose
        if (sw_cable_loose && cable_loose_state == 0) {
          motor_power = LOOSE_POWER_BRAKE;
          cable_loose_state = 1;
          set_motor_dir(3);
          time_pwm_ramp = TIME_PWM_RAMP_LOOSE_CABLE;
#if (DEBUG_SERIAL_OUT)
          Serial.println("Cable is loose");
#endif
        }
      }
      // check cable not loose anymore
      if (!sw_cable_loose && cable_loose_state == 1) {
        motor_power = START_POWER_LOOSE_CABLE;
        set_motor_dir(1);
        time_pwm_ramp = TIME_PWM_RAMP_LOOSE_CABLE;
        cable_loose_state = 0;
      }
      // LED status blinking
      if (timer_LED == LED_TIME_CLEANING) {
        digitalWrite(LED_pin, !digitalRead(LED_pin));
        timer_LED = 0;
      }
      break;
    case 2:  // winding in
      set_motor_power();
      // override function when winding in is hold pressed
      if (!button_winding_in) {
        // safty time out
        if (timer_cleaning >= TIME_MAX_WINDING_IN) {
          state_machine_main_state = 6;
        }
        // stop with pressing the other button:
        if (button_cleaning) {
          state_machine_main_state = 6;
        }
        // stop at high motor currents
        if (ADC_current_sense >= MOTOR_CURRENT_STOP) {
          state_machine_main_state = 5;
        }
      }
      // LED status blinking
      if (timer_LED >= LED_TIME_WINDING_IN) {
        digitalWrite(LED_pin, !digitalRead(LED_pin));
        timer_LED = 0;
      }
      break;
    case 5:
      set_motor_brake();
      digitalWrite(LED_pin, 0);
      break;
    case 6:
      set_motor_brake();
      digitalWrite(LED_pin, 1);
      break;
    case 7:  //stopping
      set_motor_power();
      if (motor_power == 255) {
        state_machine_main_state = 3;
        timer_cleaning = 0;
      }
      break;
  }
}
