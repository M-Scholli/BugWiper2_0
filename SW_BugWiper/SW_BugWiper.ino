#include <dummy.h>
#include <EEPROM.h>
#include <ESP32Encoder.h>

#define DUAL_MOTOR_CONTROLLER 1
#define DEBUG_SERIAL_OUT 2

// Pindiscription, follwing pins are not allowed to use: 0 (Bootselect); 2 Board LED; 1 & 3 (UART 0 for serial debug interface); 5 ?;  6, 7, 8, 9, 10 & 11 (4 MB SPI Flash); 16-17 (PSRAM)
//Button PINs
#define BUTTON_CLEANING_A_PIN 21
#define BUTTON_WINDING_IN_A_PIN 18
#define SW_CABLE_LOOSE_A_PIN 23
#if (DUAL_MOTOR_CONTROLLER)
#define BUTTON_CLEANING_B_PIN 19
#define BUTTON_WINDING_IN_B_PIN 17
#define SW_CABLE_LOOSE_B_PIN 22
#endif
#define SAFETY_SWITCH_PIN 16  // Saftyswitch to deaktivate the BugWiper
//LED configuration
#define LED_A_PIN 2
#define LED_TIME_CLEANING 500    //time blinking LED
#define LED_TIME_WINDING_IN 250  //time blinking LED
//Motor PINs
#define MOTOR_A_IN1_PIN 12
#define MOTOR_A_IN2_PIN 13
#define MOTOR_A_EN_PIN 14  //PWM Pin
#define MOTOR_A_CURRENT_SENSE_PIN 36
#define MOTOR_A_ENCODER_1_PIN 26
#define MOTOR_A_ENCODER_2_PIN 27
#define MOTOR_CURRENT_STOP 2500
#if (DUAL_MOTOR_CONTROLLER)
#define LED_B_PIN 15
#define MOTOR_B_IN1_PIN 26
#define MOTOR_B_IN2_PIN 27
#define MOTOR_B_EN_PIN 25  //PWM Pin
#define MOTOR_B_CURRENT_SENSE_PIN 39
#define MOTOR_B_ENCODER_1_PIN 34
#define MOTOR_B_ENCODER_2_PIN 35
#endif
// Kalibrierung des Putzvorganges
#define TIME_LONG_PRESS 400           //time in ms for long button press
#define MAX_POWER_CLEANING 255        //max power while cleaning
#define MAX_POWER_WINDING_IN 255      //max power while winding in
#define MAX_POWER_GROUND 255          //max power on the ground
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
//EEPROM storage erea
#define EEPROM_DIRECTION_A 0
#define EEPROM_DIRECTION_B 1
//PWM configuration
#define PWM_FREQ 10000
#define PWM_RESOLUTION_BITS 8
#define PWM_CHANNEL_A 0
#if (DUAL_MOTOR_CONTROLLER)
#define PWM_CHANNEL_B 1
#endif

//Global variables
int8_t direction_of_rotation_A;
int8_t direction_of_rotation_A_old;
int8_t cable_loose_a = 0;
uint8_t motor_power_a = 0;
uint16_t timer_button_long_press_a = 0;  // Time since start of long pressing button A
uint8_t timer_motor_power_a = 0;         //Motor PWM
uint16_t timer_LED_a = 0;                //LED
uint32_t timer_start_cleaning_a = 0;     // T_MIN , T_MAX
uint8_t pwmMax_A = 0;
uint8_t time_pwm_ramp_a = 0;
uint8_t timer_button_cable_loose_a = 0;
uint8_t timer_button_winding_in_a = 0;
uint8_t timer_button_start_cleaning_a = 0;
ESP32Encoder encoder_motor_a;

#if (DUAL_MOTOR_CONTROLLER)
int8_t direction_of_rotation_B;
int8_t direction_of_rotation_B_old;
int8_t cable_loose_b = 0;
uint8_t motor_power_b = 0;
uint16_t timer_button_long_press_b = 0;  // Time since start of long pressing button B
uint8_t timer_motor_power_b = 0;         //Motor PWM
uint16_t timer_LED_b = 0;                //LED
uint32_t timer_start_cleaning_b = 0;     // T_MIN , T_MAX
uint8_t pwmMax_B = 0;
uint8_t time_pwm_ramp_b = 0;
uint8_t timer_button_cable_loose_b = 0;
uint8_t timer_button_winding_in_b = 0;
uint8_t timer_button_start_cleaning_b = 0;
ESP32Encoder encoder_motor_b;
#endif

/* main_state machine 
 0 = ready
 1 = cleaning prozess
 2 = winding in prozess
 3 = retighten
 6 = ERROR
 7 = Stopp
 */
volatile uint8_t state_machine_main_state_a = 0;
/* sub state machine
0 = init
1 = starting
2 = running

10 =  loose cable: init /  detected
11 =  loose cable: stopping
12 = lose cable: stopped
13 = lose cable: restarting
14 = full speed reached

20 = direction change init 
21 = direction change slow down
22 = direction change full stop
23 = direction cghange new direction inti
24 = direction change restart
25 = direction change full speed reached

30 = near fuselage position reached
31 = near fuselage slow down
32 = near fuselage slow speed reached
33 = near fuselage no movemend detected
34 = near fuselage stopping

40 = stopping init
41 = stopping start
42 = stopping full stop
43 = stopping wait
44 = stopping retighten
45 = stopping stop finished
*/

volatile uint8_t state_machine_sub_state_a = 0

#if (DUAL_MOTOR_CONTROLLER)
volatile uint8_t state_machine_state_b = 0;
volatile uint8_t state_machine_sub_state_b = 0
#endif

const int pwm_a_pin = MOTOR_A_EN_PIN;
#if (DUAL_MOTOR_CONTROLLER)
const int pwm_b_pin = MOTOR_B_EN_PIN;
#endif

hw_timer_t *Timer0_Cfg = NULL;
volatile uint16_t counter_timer = 0;
volatile uint16_t ADC_current_sense_a = 0;

#if (DUAL_MOTOR_CONTROLLER)
volatile uint16_t ADC_current_sense_b = 0;
#endif



void IRAM_ATTR Timer0_ISR(void) {
  counter_timer++;
  ADC_current_sense_a = analogRead(MOTOR_A_CURRENT_SENSE_PIN);
  if (ADC_current_sense_a >= MOTOR_CURRENT_STOP) {
    switch (state_machine_state_a) {
      case 1:
        state_machine_state_a = 4;
        break;
      case 2:
        state_machine_state_a = 5;
        break;
    }
  }
#if (DUAL_MOTOR_CONTROLLER)
  ADC_current_sense_b = analogRead(MOTOR_B_CURRENT_SENSE_PIN);
  if (ADC_current_sense_b >= MOTOR_CURRENT_STOP) {
    switch (state_machine_state_b) {
      case 1:
        state_machine_state_b = 4;
        break;
      case 2:
        state_machine_state_b = 5;
        break;
    }
  }
#endif
  if (counter_timer == 10) {
    counter_timer = 0;
    setTimer();
  }
}



void Encoder_init(void) {
  encoder_motor_a.attachHalfQuad(MOTOR_A_ENCODER_1_PIN, MOTOR_A_ENCODER_2_PIN);
  encoder_motor_a.setCount(0);
#if (DUAL_MOTOR_CONTROLLER)
  encoder_motor_b.attachHalfQuad(MOTOR_B_ENCODER_1_PIN, MOTOR_B_ENCODER_2_PIN);
  encoder_motor_b.setCount(0);
#endif
}

void Timer_init(void) {
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100, true);
  timerAlarmEnable(Timer0_Cfg);
}

void PWM_inti(void) {
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pwm_a_pin, PWM_CHANNEL_A);
  ledcWrite(PWM_CHANNEL_A, 0);
#if (DUAL_MOTOR_CONTROLLER)
  ledcWrite(PWM_CHANNEL_B, 0);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pwm_b_pin, PWM_CHANNEL_B);
#endif
#if (DEBUG_SERIAL_OUT)
  Serial.println("Init PWMs complete");
#endif
}

void eeprom_update_byte(int adresse, uint8_t wert) {
  if (EEPROM.read(adresse) != wert) {
    EEPROM.write(adresse, wert);
  }
}

// switch the motor 1 = direction 1;	2 = direction 2;	3 = stop;
void motor_a(uint8_t a) {
  switch (a) {
    case 1:  // clockwise
      {
        digitalWrite(MOTOR_A_IN2_PIN, 0);
        digitalWrite(MOTOR_A_IN1_PIN, 1);
      }
      break;

    case 2:  // anticlockwise
      {
        digitalWrite(MOTOR_A_IN1_PIN, 0);
        digitalWrite(MOTOR_A_IN2_PIN, 1);
      }
      break;

    case 3:  // stop
      {
        digitalWrite(MOTOR_A_IN2_PIN, 0);
        digitalWrite(MOTOR_A_IN1_PIN, 0);
      }
      break;
  }
}

#if (DUAL_MOTOR_CONTROLLER)
void motor_b(uint8_t a) {
  switch (a) {
    case 1:  // clockwise
      {
        digitalWrite(MOTOR_B_IN2_PIN, 0);
        digitalWrite(MOTOR_B_IN1_PIN, 1);
      }
      break;

    case 2:  // anticlockwise
      {
        digitalWrite(MOTOR_B_IN1_PIN, 0);
        digitalWrite(MOTOR_B_IN2_PIN, 1);
      }
      break;

    case 3:  // stop
      {
        digitalWrite(MOTOR_B_IN2_PIN, 0);
        digitalWrite(MOTOR_B_IN1_PIN, 0);
      }
      break;
  }
}
#endif

void set_motorpower_a(void) {
  if (timer_motor_power_a >= time_pwm_ramp_a) {
    if (motor_power_a < pwmMax_A) {
      motor_power_a++;
    }
    timer_motor_power_a = 0;
  }
  ledcWrite(PWM_CHANNEL_A, motor_power_a);
}

#if (DUAL_MOTOR_CONTROLLER)
void set_motorpower_b(void) {
  if (timer_motor_power_b >= time_pwm_ramp_b) {
    if (motor_power_b < pwmMax_B) {
      motor_power_b++;
    }
    timer_motor_power_b = 0;
  }
  ledcWrite(PWM_CHANNEL_B, motor_power_b);
}
#endif

void set_motor_brake_a(void) {
  timer_motor_power_a = 0;
  motor_power_a = START_POWER_BRAKE;
  time_pwm_ramp_a = TIME_PWM_RAMP_BRAKE;
  pwmMax_A = MAX_POWER_BRAKE;
  motor_a(3);
  state_machine_state_a = 7;
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start braking A");
#endif
}

#if (DUAL_MOTOR_CONTROLLER)
void set_motor_brake_b(void) {
  timer_motor_power_b = 0;
  motor_power_b = START_POWER_BRAKE;
  time_pwm_ramp_b = TIME_PWM_RAMP_BRAKE;
  pwmMax_B = MAX_POWER_BRAKE;
  motor_b(3);
  state_machine_state_b = 7;
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start braking B");
#endif
}
#endif

void set_winding_in_a(void) {
  state_machine_state_a = 2;
  timer_LED_a = 0;
  timer_motor_power_a = 0;
  timer_start_cleaning_a = 0;
  motor_power_a = START_POWER_WINDING_IN;
  time_pwm_ramp_a = TIME_PWM_RAMP_WINDING_IN;
  pwmMax_A = MAX_POWER_WINDING_IN;
  if (direction_of_rotation_A == 1) {
    motor_a(2);
    direction_of_rotation_A_old = 2;
  } else {
    motor_a(1);
    direction_of_rotation_A_old = 1;
  }
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start winding in A");
#endif
}

#if (DUAL_MOTOR_CONTROLLER)
void set_winding_in_b(void) {
  state_machine_state_b = 2;
  timer_LED_b = 0;
  timer_motor_power_b = 0;
  timer_start_cleaning_b = 0;
  motor_power_b = START_POWER_WINDING_IN;
  time_pwm_ramp_b = TIME_PWM_RAMP_WINDING_IN;
  pwmMax_B = MAX_POWER_WINDING_IN;
  if (direction_of_rotation_B == 1) {
    motor_b(2);
    direction_of_rotation_B_old = 2;
  } else {
    motor_b(1);
    direction_of_rotation_B_old = 1;
  }
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start winding in B");
#endif
}
#endif

void set_start_cleaning_a(void) {
  state_machine_state_a = 1;
  timer_motor_power_a = 0;
  timer_LED_a = 0;
  timer_start_cleaning_a = 0;
  motor_power_a = START_POWER_CLEANING;
  time_pwm_ramp_a = TIME_PWM_RAMP_CLEANING;
  pwmMax_A = MAX_POWER_CLEANING;
  motor_a(direction_of_rotation_A);
  timer_button_long_press_a = 0;
  direction_of_rotation_A_old = direction_of_rotation_A;
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start cleaning A");
#endif
}

#if (DUAL_MOTOR_CONTROLLER)
void set_start_cleaning_b(void) {
  state_machine_state_b = 1;
  timer_motor_power_b = 0;
  timer_LED_b = 0;
  timer_start_cleaning_b = 0;
  motor_power_b = START_POWER_CLEANING;
  time_pwm_ramp_b = TIME_PWM_RAMP_CLEANING;
  pwmMax_B = MAX_POWER_CLEANING;
  motor_b(direction_of_rotation_B);
  timer_button_long_press_b = 0;
  direction_of_rotation_B_old = direction_of_rotation_B;
#if (DEBUG_SERIAL_OUT)
  Serial.println("Start cleaning B");
#endif
}
#endif

void init_io(void) {
#if (DEBUG_SERIAL_OUT)
  Serial.println("Init PINs A:");
#endif
  pinMode(SW_CABLE_LOOSE_A_PIN, INPUT_PULLUP);
  pinMode(SAFETY_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(BUTTON_WINDING_IN_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CLEANING_A_PIN, INPUT_PULLUP);
  digitalWrite(MOTOR_A_IN2_PIN, 1);
  digitalWrite(MOTOR_A_IN1_PIN, 1);
  digitalWrite(LED_A_PIN, 0);
#if (DEBUG_SERIAL_OUT)
  Serial.println("Init PINs A complete");
#endif
#if (DUAL_MOTOR_CONTROLLER)
#if (DEBUG_SERIAL_OUT)
  Serial.println("Init PINs B:");
#endif
  pinMode(SW_CABLE_LOOSE_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B_IN1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN2_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(BUTTON_WINDING_IN_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CLEANING_B_PIN, INPUT_PULLUP);
  digitalWrite(MOTOR_B_IN2_PIN, 1);
  digitalWrite(MOTOR_B_IN1_PIN, 1);
  digitalWrite(LED_B_PIN, 0);
#endif
}

void EEPROM_read_directions(void) {
  direction_of_rotation_A = EEPROM.read(EEPROM_DIRECTION_A);
  if (direction_of_rotation_A != 1 && direction_of_rotation_A != 2) {
    direction_of_rotation_A = 1;
    eeprom_update_byte(EEPROM_DIRECTION_A, direction_of_rotation_A);
#if (DEBUG_SERIAL_OUT)
    Serial.println("EEPROM Error: Reset rotation direction A to 1");
#endif
  }
#if (DUAL_MOTOR_CONTROLLER)
  direction_of_rotation_B = EEPROM.read(EEPROM_DIRECTION_B);
  if (direction_of_rotation_B != 1 && direction_of_rotation_B != 2) {
    direction_of_rotation_B = 1;
    eeprom_update_byte(EEPROM_DIRECTION_B, direction_of_rotation_B);
#if (DEBUG_SERIAL_OUT)
    Serial.println("EEPROM Error: Reset rotation direction B to 1");
#endif
  }
#endif
#if (DEBUG_SERIAL_OUT)
  Serial.println("EEPROM read direction finished");
#endif
}

void EEPROM_write_directions(void) {
  eeprom_update_byte(EEPROM_DIRECTION_A, direction_of_rotation_A);
#if (DUAL_MOTOR_CONTROLLER)
  eeprom_update_byte(EEPROM_DIRECTION_B, direction_of_rotation_B);
#endif
}

void change_direction_a(void) {
  if (direction_of_rotation_A == 1) {
    direction_of_rotation_A = 2;
  } else if (direction_of_rotation_A == 2) {
    direction_of_rotation_A = 1;
  }
  EEPROM_write_directions();
}

#if (DUAL_MOTOR_CONTROLLER)
void change_direction_b(void) {
  if (direction_of_rotation_B == 1) {
    direction_of_rotation_B = 2;
  } else if (direction_of_rotation_B == 2) {
    direction_of_rotation_B = 1;
  }
  EEPROM_write_directions();
}
#endif

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
#if (DUAL_MOTOR_CONTROLLER)
  if (digitalRead(SW_CABLE_LOOSE_B_PIN) == 0 && timer_button_cable_loose_b < 255) {
    timer_button_cable_loose_b++;
  } else if (digitalRead(SW_CABLE_LOOSE_B_PIN) && timer_button_cable_loose_b > 0) {
    timer_button_cable_loose_b--;
  }
  if (digitalRead(BUTTON_WINDING_IN_B_PIN) == 0 && timer_button_winding_in_b < 255) {
    timer_button_winding_in_b++;
  } else if (digitalRead(BUTTON_WINDING_IN_B_PIN) && timer_button_winding_in_b > 0) {
    timer_button_winding_in_b--;
  }
  if (digitalRead(BUTTON_CLEANING_B_PIN) == 0 && timer_button_start_cleaning_b < 255) {
    timer_button_start_cleaning_b++;
  } else if (digitalRead(BUTTON_CLEANING_B_PIN) && timer_button_start_cleaning_b > 0) {
    timer_button_start_cleaning_b--;
  }
#endif
}

//counts every ms the timer one higher
void setTimer(void) {
  timer_start_cleaning_a++;
  timer_motor_power_a++;
  timer_LED_a++;
  if (timer_button_start_cleaning_a >= TIME_BUTTON_DEBOUNCE && state_machine_state_a == 0) {
    timer_button_long_press_a = timer_button_long_press_a + 1;
  }
#if (DUAL_MOTOR_CONTROLLER)
  timer_start_cleaning_b++;
  timer_motor_power_b++;
  timer_LED_b++;
  if (timer_button_start_cleaning_b >= TIME_BUTTON_DEBOUNCE && state_machine_state_b == 0) {
    timer_button_long_press_b = timer_button_long_press_b + 1;
  }
#endif
  button_debounce();
}

void read_Buttons(void) {
  if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
    if (timer_button_winding_in_a >= TIME_BUTTON_DEBOUNCE && state_machine_state_a == 0) {
      set_winding_in_a();
    }
    if (state_machine_state_a == 0 && timer_button_long_press_a >= TIME_LONG_PRESS) {
      set_start_cleaning_a();
    }
#if (DUAL_MOTOR_CONTROLLER)
    if (timer_button_winding_in_b >= TIME_BUTTON_DEBOUNCE && state_machine_state_b == 0) {
      set_winding_in_b();
    }
    if (state_machine_state_b == 0 && timer_button_long_press_b >= TIME_LONG_PRESS) {
      set_start_cleaning_b();
    }
#endif
  }
  // prevents a imediate second start cleaning after finish the first one
  if (timer_button_winding_in_a <= 5 && timer_button_start_cleaning_a <= 5
      && state_machine_state_a == 3 && timer_start_cleaning_a >= 200) {
    state_machine_state_a = 0;
  }
#if (DUAL_MOTOR_CONTROLLER)
  if (timer_button_winding_in_b <= 5 && timer_button_start_cleaning_b <= 5
      && state_machine_state_b == 3 && timer_start_cleaning_b >= 200) {
    state_machine_state_b = 0;
  }
#endif
  // reset timer for long press of buttons
  if (timer_button_start_cleaning_a <= 5) {
    timer_button_long_press_a = 0;
  }
#if (DUAL_MOTOR_CONTROLLER)
  if (timer_button_start_cleaning_b <= 5) {
    timer_button_long_press_b = 0;
  }
#endif
}

void check_end(void) {
  if (state_machine_state_a == 4) {
    set_motor_brake_a();
    change_direction_a();
    digitalWrite(LED_A_PIN, 0);
  }
  if (state_machine_state_a == 5) {
    set_motor_brake_a();
    digitalWrite(LED_A_PIN, 0);
  }
  if (state_machine_state_a == 6) {
    set_motor_brake_a();
    digitalWrite(LED_A_PIN, 1);
  }
#if (DUAL_MOTOR_CONTROLLER)
  if (state_machine_state_b == 4) {
    set_motor_brake_b();
    change_direction_b();
    digitalWrite(LED_B_PIN, 0);
  }
  if (state_machine_state_b == 5) {
    set_motor_brake_b();
    digitalWrite(LED_B_PIN, 0);
  }
  if (state_machine_state_b == 6) {
    set_motor_brake_b();
    digitalWrite(LED_B_PIN, 1);
  }
#endif
}

void state_machine_motor_a(void) {
  switch (state_machine_state_a) {
    case 0:  // do nothing
      break;
    case 1:  // cleaning
      set_motorpower_a();
      // override function when start cleaning is hold pressed
      if (timer_button_start_cleaning_a <= TIME_BUTTON_DEBOUNCE) {
        // safty time out
        if (timer_start_cleaning_a >= TIME_MAX_CLEANING) {
          state_machine_state_a = 6;
        }
        // stop with pressing the other button:
        if (timer_button_winding_in_a >= TIME_BUTTON_DEBOUNCE) {
          state_machine_state_a = 6;
        }
        // check cable loose
        if (timer_button_cable_loose_a <= 5 && cable_loose_a == 0) {
          motor_power_a = LOOSE_POWER_BRAKE;
          cable_loose_a = 1;
          motor_a(3);
          time_pwm_ramp_a = TIME_PWM_RAMP_LOOSE_CABLE;
#if (DEBUG_SERIAL_OUT)
          Serial.println("Cable is loose");
#endif
        }
      }
      // check cable not loose anymore
      if (timer_button_cable_loose_a >= TIME_BUTTON_DEBOUNCE && cable_loose_a == 1) {
        motor_power_a = START_POWER_LOOSE_CABLE;
        motor_a(direction_of_rotation_A_old);
        time_pwm_ramp_a = TIME_PWM_RAMP_LOOSE_CABLE;
        cable_loose_a = 0;
      }
      // LED status blinking
      if (timer_LED_a == LED_TIME_CLEANING) {
        digitalWrite(LED_A_PIN, !digitalRead(LED_A_PIN));
        timer_LED_a = 0;
      }
      break;
    case 2:  // winding in
      set_motorpower_a();
      if (timer_button_winding_in_a <= TIME_BUTTON_DEBOUNCE) {
        // maximale Einziehzeit erreicht
        if (timer_start_cleaning_a >= TIME_MAX_WINDING_IN) {
          state_machine_state_a = 6;
        }
        //Stopp bei drücken des Putzen Pins
        if (timer_button_start_cleaning_a >= TIME_BUTTON_DEBOUNCE) {
          state_machine_state_a = 6;
        }
        if (timer_start_cleaning_a >= TIME_MIN_CLEANING && timer_button_cable_loose_a <= 5 && cable_loose_a == 0) {
          motor_power_a = LOOSE_POWER_BRAKE;
          cable_loose_a = 1;
          motor_a(3);
          time_pwm_ramp_a = TIME_PWM_RAMP_LOOSE_CABLE;
#if (DEBUG_SERIAL_OUT)
          Serial.println("Cable is loose");
#endif
        }
      }
      if (timer_LED_a >= LED_TIME_WINDING_IN) {
        digitalWrite(LED_A_PIN, !digitalRead(LED_A_PIN));
        timer_LED_a = 0;
      }
      break;
    case 7:  //stopping
      set_motorpower_a();
      if (motor_power_a == 255) {
        state_machine_state_a = 3;
        timer_start_cleaning_a = 0;
      }
      break;
  }

  if (timer_button_cable_loose_a >= TIME_BUTTON_DEBOUNCE && cable_loose_a == 1) {
    motor_power_a = START_POWER_LOOSE_CABLE;
    motor_a(direction_of_rotation_A_old);
    time_pwm_ramp_a = TIME_PWM_RAMP_LOOSE_CABLE;
    cable_loose_a = 0;
  }
  // Motor bremsen
}

#if (DUAL_MOTOR_CONTROLLER)
void state_machine_motor_b(void) {
  if (state_machine_state_b == 1 || state_machine_state_b == 2 || state_machine_state_b == 7) {
    set_motorpower_b();
  }
  if (state_machine_state_b == 1) {
    if (timer_button_start_cleaning_b <= TIME_BUTTON_DEBOUNCE) {
      if (timer_start_cleaning_b >= TIME_MAX_CLEANING) {
        state_machine_state_b = 6;
      }
      if (timer_button_winding_in_b >= TIME_BUTTON_DEBOUNCE) {
        state_machine_state_b = 6;
      }
      if (timer_start_cleaning_b >= TIME_MIN_CLEANING && timer_button_cable_loose_b <= 5 && cable_loose_b == 0) {
        motor_power_b = LOOSE_POWER_BRAKE;
        motor_b(3);
        time_pwm_ramp_b = TIME_PWM_RAMP_LOOSE_CABLE;
        cable_loose_b = 1;
      }
    }
    if (timer_LED_b == LED_TIME_CLEANING) {
      digitalWrite(LED_B_PIN, !digitalRead(LED_B_PIN));
      timer_LED_b = 0;
    }
    if (timer_button_cable_loose_b >= TIME_BUTTON_DEBOUNCE && cable_loose_b == 1) {
      motor_power_b = START_POWER_LOOSE_CABLE;
      motor_b(direction_of_rotation_B_old);
      time_pwm_ramp_b = TIME_PWM_RAMP_LOOSE_CABLE;
      cable_loose_b = 0;
    }
  }
  if (state_machine_state_b == 2) {
    if (timer_button_winding_in_b <= TIME_BUTTON_DEBOUNCE) {
      // maximale Einziehzeit erreicht
      if (timer_start_cleaning_b >= TIME_MAX_WINDING_IN) {
        state_machine_state_b = 6;
      }
      //Stopp bei drücken des Putzen Pins
      if (timer_button_start_cleaning_b >= TIME_BUTTON_DEBOUNCE) {
        state_machine_state_b = 6;
      }
      if (timer_start_cleaning_b >= TIME_MIN_CLEANING && timer_button_cable_loose_b <= 5 && cable_loose_b == 0) {
        motor_power_b = LOOSE_POWER_BRAKE;
        motor_b(3);
        time_pwm_ramp_b = TIME_PWM_RAMP_LOOSE_CABLE;
        cable_loose_b = 1;
      }
    }
    // LED Blinken
    if (timer_LED_b >= LED_TIME_WINDING_IN) {
      digitalWrite(LED_B_PIN, !digitalRead(LED_B_PIN));
      timer_LED_b = 0;
    }
    if (timer_button_cable_loose_b >= TIME_BUTTON_DEBOUNCE && cable_loose_b == 1) {
      motor_power_b = START_POWER_LOOSE_CABLE;
      motor_b(direction_of_rotation_B_old);
      time_pwm_ramp_b = TIME_PWM_RAMP_LOOSE_CABLE;
      cable_loose_b = 0;
    }
  }
  if (state_machine_state_b == 7 && motor_power_b == 255) {
    state_machine_state_b = 3;
    timer_start_cleaning_b = 0;
  }
}
#endif

//The setup function is called once at startup of the sketch
void setup() {
#if (DEBUG_SERIAL_OUT)
  Serial.begin(115200);
  Serial.println("BugWiper start programm");
#endif
  Encoder_init();
  PWM_inti();
  init_io();
  EEPROM_read_directions();
  Timer_init();
#if (DEBUG_SERIAL_OUT)
  if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
    Serial.println("SAFETY SWITCH closed");
  } else {
    Serial.println("WARNING!!! SAFETY SWITCH open");
    Serial.println("Close the SAFETY SWITCH to operate the BugWiper");
  }
#endif
}

// The loop function is called in an endless loop
void loop() {
  read_Buttons();
  state_machine_motor_a();
#if (DUAL_MOTOR_CONTROLLER)
  state_machine_motor_b();
#endif
  check_end();  // cleaning finished?
#if (DEBUG_SERIAL_OUT >= 2)
  Serial.println("ADC value = " + String(ADC_current_sense_a));
  Serial.println("Encoder count = " + String((int32_t)encoder_motor_a.getCount()));
#if (DUAL_MOTOR_CONTROLLER)
  Serial.println("ADC_B value = " + String(ADC_current_sense_b));
  Serial.println("Encoder_B count = " + String((int32_t)encoder_motor_b.getCount()));
#endif
#endif
}
