#include "esp32-hal-gpio.h"
#include "esp32-hal-adc.h"
#include "esp32-hal-ledc.h"
#include <sys/_stdint.h>
#include <Arduino.h>
#include "my_debug.h"
#include "BugWiper.h"
#include "btn99x0_motor_control.hpp"

#define ADC_FILTER_SIZE 32
uint32_t BW_ADC_current_sense;
uint32_t ADC_current_filter_sum;
uint32_t ADC_current_old_values[ADC_FILTER_SIZE];
int8_t   ADC_filter_counter = 0;
uint32_t BW_ADC_current_filtered;

double BW_ADC_current_mA;
uint16_t BW_ADC_current_counter;
volatile double BW_ADC_current_mA_filtered;
float BW_ADC_T_ntc_degree;
float BW_ADC_V_Bat;
volatile double BW_ADC_btn_hb1;
volatile double BW_ADC_btn_hb2;

uint16_t BW_state_machine_state;
volatile uint32_t BW_state_machine_timer;
volatile uint32_t BW_state_machine_timer_2;


volatile int64_t motor_enc_count;  // counts from encoder
volatile int32_t BW_position;      // position in mm converted from the encoder
volatile int32_t BW_speed;
int16_t BW_speed_counter;

enum BW_MODE BW_mode = M_IDLE;

uint16_t timer_button_cable_loose = 0;
uint16_t timer_button_winding_in = 0;
uint16_t timer_button_start_cleaning = 0;
uint16_t timer_button_long_press = 0;  // Time since start of long pressing button

bool button_winding_in;
bool button_start_cleaning;
bool button_cable_loose;

int motor_pwm_channel;
gpio_num_t motor_pwm_pin;

uint8_t motor_power;
uint8_t motor_power_dest;
volatile uint16_t timer_LED;
uint16_t LED_time;
uint8_t time_pwm_ramp;
volatile uint8_t timer_motor_power;

bool motor_inverted;
bool cable_loose;
enum direction motor_direction;

ESP32Encoder BW_motor_encoder;

using namespace btn99x0;

io_pins_t hb1_io_pins = {
  MOTOR_IS1_PIN,
  MOTOR_IN1_PIN,
  MOTOR_INH1_PIN
};

io_pins_t hb2_io_pins = {
  MOTOR_IS2_PIN,
  MOTOR_IN2_PIN,
  MOTOR_INH2_PIN
};

hw_conf_t hw_conf = {
  1000,  // Rsense in Ohm
  3.1,  // VOltage Range
  8191   // ADC Steps
};

DCShield shield(hb1_io_pins, hb2_io_pins, hw_conf);
MotorControl btn_motor_control(shield);
HalfBridge HalfBridge_1 = shield.get_half_bridge(DCShield::HALF_BRIDGE_1);
HalfBridge HalfBridge_2 = shield.get_half_bridge(DCShield::HALF_BRIDGE_2);

void Encoder_init(void) {
  BW_motor_encoder.attachHalfQuad(MOTOR_ENCODER_1_PIN, MOTOR_ENCODER_2_PIN);
  BW_motor_encoder.setCount(0);
}

void rgbLedWrite_colour(struct RBG_COLOUR colour) {
  rgbLedWrite(RGB_LED_PIN,colour.g, colour.r, colour.b);
}

// TEST Functions
void BugWiper_test_LED(void) {
  DEBUG_INFO("Test LEDs")

  for (uint8_t i=0; i<5; i++) {
    rgbLedWrite_colour(ModeLED_Colour[i]);
    delay(500);
  }
  rgbLedWrite_colour(ModeLED_Colour[0]);
}

void BugWiper_test_Motor(void) {
  DEBUG_INFO("Run forward for 2 sec...")
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));
  rgbLedWrite(RGB_LED_PIN, RGB_BRIGHTNESS, 0, 0);  // Red
  btn_motor_control.set_speed(180);
  delay(500);
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));
  delay(500);
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));
  DEBUG_INFO("Load current (A): ");
  DEBUG_INFO(HalfBridge_1.get_load_current_in_amps());
  DEBUG_INFO(HalfBridge_2.get_load_current_in_amps());
  DEBUG_INFO(((float)analogReadMilliVolts(MOTOR_CURRENT_SENSE_PIN) * 0.005));
  delay(1000);
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));

  DEBUG_INFO("Freewheel for 1 sec...");
  rgbLedWrite(RGB_LED_PIN, 0, RGB_BRIGHTNESS, 0);  // Green
  btn_motor_control.freewheel();
  delay(1000);
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));

  DEBUG_INFO("Run backward for 2 sec...");
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));
  rgbLedWrite(RGB_LED_PIN, 0, 0, RGB_BRIGHTNESS);  // Blue
  btn_motor_control.set_speed(-180);
  delay(500);
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));
  delay(500);
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));
  DEBUG_INFO("Load current (A): ");
  DEBUG_INFO(HalfBridge_1.get_load_current_in_amps());
  DEBUG_INFO(HalfBridge_2.get_load_current_in_amps());
  DEBUG_INFO(((float)analogReadMilliVolts(MOTOR_CURRENT_SENSE_PIN) * 0.005));
  delay(1000);
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));

  DEBUG_INFO("Brake for 1 sec...");
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);  // Blue
  btn_motor_control.brake();
  delay(1000);
  motor_enc_count = BW_motor_encoder.getCount();
  DEBUG_INFO("Encoder count = " + String((int32_t)BW_motor_encoder.getCount()));
}

// switch the motor to out, in or stops
void BugWiper_set_motor_dir(enum direction dir) {
  motor_direction = dir;
#ifdef BTS7960B_CONTROLLER
  switch (dir) {
    case OUT:  // out
      digitalWrite(MOTOR_IN2_PIN, 0);
      digitalWrite(MOTOR_IN1_PIN, 1);
      break;

    case IN:  // in
      digitalWrite(MOTOR_IN1_PIN, 0);
      digitalWrite(MOTOR_IN2_PIN, 1);
      break;

    case STOP:  // stop
      digitalWrite(MOTOR_IN2_PIN, 0);
      digitalWrite(MOTOR_IN1_PIN, 0);
      break;
  }
#endif
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
#ifdef BTN9960_CONTROLLER
  switch (motor_direction) {
    case OUT:
      btn_motor_control.set_speed(-motor_power);
      break;
    case IN:
      btn_motor_control.set_speed(motor_power);
      break;
    case STOP:
      btn_motor_control.brake();
      break;
    case Freewheeling:
      btn_motor_control.freewheel();
      break;
    default:
      btn_motor_control.brake();
      break;
  }

#elif defined(BTS7960B_CONTROLLER)
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Code for version 3.x
  ledcWrite(motor_pwm_pin, motor_power);
#else
  // Code for version 2.x
  ledcWrite(motor_pwm_channel, motor_power);
#endif
#endif
}

void BugWiper_LED_blinking(void) {
  if (timer_LED >= LED_time) {
    //digitalWrite(RGB_LED_PIN, !digitalRead(RGB_LED_PIN)); Fixme
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
  BW_state_machine_state = BW_STATE_START_WINDING_IN + 1;
  timer_LED = 0;
  timer_motor_power = 0;
  BW_state_machine_timer = 0;
  BW_state_machine_timer_2 = 0;
  motor_power = START_POWER_WINDING_IN;
  time_pwm_ramp = TIME_PWM_RAMP_WINDING_IN;
  motor_power_dest = MAX_POWER_WINDING_IN;
  BugWiper_set_motor_dir(IN);
  BW_mode = M_WINDING_IN;
  rgbLedWrite_colour(ModeLED_Colour[BW_mode]);
  DEBUG_INFO("Start winding in");
}

void BugWiper_set_start_cleaning(void) {
  BW_motor_encoder.setCount(0);
  BW_state_machine_state = BW_STATE_START_CLEANING;
  BW_state_machine_timer = 0;
  BW_state_machine_timer_2 = 0;
  BW_mode = M_CLEANING;
  rgbLedWrite_colour(ModeLED_Colour[BW_mode]);
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
      BW_mode = M_IDLE;
      rgbLedWrite_colour(ModeLED_Colour[BW_mode]);
      BW_state_machine_state++;
      break;
    case 1: //wait
      break;
    // 1x starting winding out
    case 10:
      timer_motor_power = 0;
      timer_LED = 0;
      BW_state_machine_timer = 0;
      BW_state_machine_timer_2 = 0;
      motor_power = START_POWER_CLEANING;
      time_pwm_ramp = TIME_PWM_RAMP_START;
      motor_power_dest = MAX_POWER_START_CLEANING;
      BugWiper_set_motor_dir(OUT);
      BW_state_machine_state++;
      break;
    case 11: // slow start cleaning
      if (BW_position > POSITION_STARTING) {
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
    case 21: // fast cleaning
      if (BW_position > (POSITION_SLOW_WINGTIP)) {
        BW_state_machine_state = 40;
      }
      if (cable_loose) {
        BW_state_machine_state = 30;
      }
      break;
    case 30: // stop if cable is loose
      BugWiper_set_motor_dir(STOP);
      motor_power = LOOSE_POWER_BRAKE;
      motor_power_dest = LOOSE_POWER_BRAKE;
      BW_state_machine_state++;
      DEBUG_INFO("Cable is loose");
      break;
    case 31:
      if (!cable_loose) {
        if (BW_position < POSITION_STARTING) {
          BW_state_machine_state = 10;
        } else {
          motor_power = START_POWER_CLEANING;
          BugWiper_set_motor_dir(OUT);
          BW_state_machine_state = 20;
        }
      }
      break;
    case 40: // slow bevore wingtip
      motor_power_dest = 0;
      time_pwm_ramp = TIME_PWN_RAMP_SLOW;
      BW_state_machine_state++;
      break;
    case 41:
      if (BW_position > POSITION_WINGTIP || motor_power <= 10) {
        BW_state_machine_state = 50;
      }
      if (cable_loose) {
        BW_state_machine_state = 50;
      }
      break;
    case 50:
        DEBUG_INFO("Wingtip reached")
        BugWiper_set_motor_brake();
        BW_state_machine_timer_2 = 0;
    case 51:
      if(BW_state_machine_timer_2 > 250) {
        BW_state_machine_state = BW_STATE_START_WINDING_IN;
      }
    case BW_STATE_START_WINDING_IN:
      timer_motor_power = 0;
      timer_LED = 0;
      //motor_power = START_POWER_WINDING_IN;
      time_pwm_ramp = TIME_PWM_RAMP_START;
      motor_power_dest = MAX_POWER_WINDING_IN;
      BugWiper_set_motor_dir(IN);
      BW_state_machine_state++;
      break;
    case 61: // winding in fast
      if (BW_position < LENGTH_SLOW) {
        BW_state_machine_state = 70;
      }
      break;
    case 70:
      motor_power_dest = MAX_POWER_NEAR_END;
      time_pwm_ramp = TIME_PWN_RAMP_SLOW;
      BW_state_machine_state++;
      break;
    case 71: // winding in slow
      if (1) {
        //BW_state_machine_state = 80;
      }
      break;
    case 80:
      BugWiper_set_motor_brake();
      BW_state_machine_timer = 0;
      BW_mode = M_FINISHED;
      rgbLedWrite_colour(ModeLED_Colour[BW_mode]);
      BW_state_machine_state++;
      break;
    case 81:
      if (BW_state_machine_timer >= TIME_FINISH_RESET){
        BW_state_machine_state = 0;
      }
      break;
    case 100: // STOP FUNCTION
      DEBUG_ERROR("STOP BUGWIPER ERORR")
      BW_state_machine_timer = 0;
      BugWiper_set_motor_brake();
      BW_state_machine_state++;
      break;
    case 101:
      if (BW_state_machine_timer >= TIME_ERROR_RESET){
        BW_state_machine_state = 0;
      }
      break;
  }
}

void BugWiper_read_motor_current(void) {
  BW_ADC_current_sense = analogReadMilliVolts(MOTOR_CURRENT_SENSE_PIN);
  BW_ADC_current_mA = BW_ADC_current_sense * CURRENT_CAL_FACTOR;
  BW_ADC_btn_hb1 = HalfBridge_1.get_load_current_in_amps();
  BW_ADC_btn_hb2 = HalfBridge_2.get_load_current_in_amps();
}

void BugWiper_ADC_filter_init(void) {
  BW_ADC_current_sense = analogReadMilliVolts(MOTOR_CURRENT_SENSE_PIN);
  ADC_current_filter_sum = 0;
  uint8_t j;
  for (j = 0; j < ADC_FILTER_SIZE; j++) {
    ADC_current_old_values[j] = BW_ADC_current_sense;
    ADC_current_filter_sum += BW_ADC_current_sense;
  }
}

void BugWiper_ADC_filter(void) {
	uint8_t i;
  ADC_current_filter_sum -= ADC_current_old_values[ADC_filter_counter];
  ADC_current_old_values[ADC_filter_counter] = BW_ADC_current_sense;
  ADC_current_filter_sum += ADC_current_old_values[ADC_filter_counter];
  BW_ADC_current_filtered = ADC_current_filter_sum / ADC_FILTER_SIZE;
  BW_ADC_current_mA_filtered = BW_ADC_current_filtered * CURRENT_CAL_FACTOR;
	ADC_filter_counter++;
	if (ADC_filter_counter >= ADC_FILTER_SIZE) {
		ADC_filter_counter = 0;
	}
}

void BugWiper_read_ADCs_slow(void) {
  uint16_t adc_temp;
  adc_temp = analogReadMilliVolts(ADC_NTC_PIN);
  BW_ADC_T_ntc_degree = (float)adc_temp * 4095.0 / 3100.0;
  adc_temp = (uint16_t) BW_ADC_T_ntc_degree;
  BW_ADC_T_ntc_degree = ((float)adc_temp*(float)adc_temp*(float)adc_temp*(-2.87638e-9))+((float)adc_temp*(float)adc_temp*(2.01243e-5))+((-0.0702)*(float)adc_temp)+109.013;
  BW_ADC_V_Bat=analogReadMilliVolts(ADC_VBat_PIN)*0.0081;
}

void BugWiper_check_end_reached(void){
  if (BW_ADC_current_mA_filtered >= BW_STOP_CURRENT)
  {
    BW_ADC_current_counter++;
    if (BW_ADC_current_counter > BW_STOP_CURRENT_COUNTS) {
      DEBUG_INFO("Finished: current:" + String(BW_ADC_current_mA) + " above " + String((float)BW_STOP_CURRENT));
      BW_state_machine_state = BW_STATE_FINISHED;
      BW_mode = M_FINISHED;
    }
  } else {
    if (BW_ADC_current_counter > 0) {
      BW_ADC_current_counter--;
    }
  }
  if (BW_mode == M_CLEANING || BW_mode == M_WINDING_IN)
  {
    if (BW_state_machine_state > BW_STATE_CHECK_END && abs(BW_speed) < BW_STOP_SPEED && BW_state_machine_timer > TIME_MIN_CLEANING)
    {
      BW_speed_counter++;
      if(BW_speed_counter >= BW_STOP_SPEED_COUNTS) {
        DEBUG_INFO("Finished: Speed:" + String(abs(BW_speed)) + " below " + String((float)BW_STOP_SPEED));
        BW_state_machine_state = BW_STATE_FINISHED;
        BW_mode = M_FINISHED;
       }
    } else {
      if (BW_speed_counter > 1) {
        BW_speed_counter--;
      }
    }
    if (BW_ADC_V_Bat <= BW_STOP_V_BAT) 
    {
      DEBUG_ERROR("Under Voltage: V BAT:" + String(BW_ADC_V_Bat) + " below " + String((float)BW_STOP_V_BAT) + "V");
      BW_state_machine_state = BW_STATE_ERROR;
      BW_mode = M_ERROR;
    }
    if (BW_ADC_T_ntc_degree > BW_STOP_T_MAX)
    {
      DEBUG_ERROR("Over Temperature: T NTC:" + String(BW_ADC_T_ntc_degree) + "above " + String((float)BW_STOP_T_MAX) + "DEG");
      BW_state_machine_state = BW_STATE_ERROR;
      BW_mode = M_ERROR;
    }
  }
}

void BugWiper_set_timer(void) {
  BW_state_machine_timer ++;
  BW_state_machine_timer_2 ++;
  timer_motor_power ++;
  timer_LED ++;
}

void BugWiper_read_Encoder(void) {
  uint32_t BW_enc_count_old = motor_enc_count;
  motor_enc_count = BW_motor_encoder.getCount();
  BW_position = int32_t((float)motor_enc_count * SPOOL_CIRCUMFERENCE / (CPR_Encoder * GEAR_RATIO));
  BW_speed= motor_enc_count - BW_enc_count_old;
}

void button_debounce(void) {
  button_cable_loose = digitalRead(SW_CABLE_LOOSE_PIN);
  if (button_cable_loose == 0 && timer_button_cable_loose < 255) {
    timer_button_cable_loose++;
  } else if (button_cable_loose && timer_button_cable_loose > 0) {
    timer_button_cable_loose--;
  }
  button_winding_in = digitalRead(BUTTON_WINDING_IN_PIN);
  if (button_winding_in == 0 && timer_button_winding_in < 255) {
    timer_button_winding_in++;
  } else if (button_winding_in && timer_button_winding_in > 0) {
    timer_button_winding_in--;
  }
  button_start_cleaning = digitalRead(BUTTON_CLEANING_PIN);
  if (button_start_cleaning == 0 && timer_button_start_cleaning < 255) {
    timer_button_start_cleaning++;
  } else if (button_start_cleaning && timer_button_start_cleaning > 0) {
    timer_button_start_cleaning--;
  }
}

void read_Buttons(void) {
  if (BW_mode==M_IDLE) {  // FIXME safety pin
    if (timer_button_winding_in >= TIME_BUTTON_DEBOUNCE) {
      BugWiper_set_winding_in();
    }
    if (timer_button_start_cleaning >= TIME_BUTTON_DEBOUNCE) {
      BugWiper_set_start_cleaning();
    }
  }
  if (BW_mode==M_CLEANING) {
    if (timer_button_winding_in >= TIME_BUTTON_DEBOUNCE) {
      BW_mode=M_STOP;
      BW_state_machine_state = BW_STATE_STOP;
      rgbLedWrite_colour(ModeLED_Colour[BW_mode]);
    }
  }
  if (BW_mode==M_WINDING_IN) {
    if (timer_button_start_cleaning >= TIME_BUTTON_DEBOUNCE) {
      BW_mode=M_STOP;
      BW_state_machine_state = BW_STATE_STOP;
      rgbLedWrite_colour(ModeLED_Colour[BW_mode]);
    }
  }
  // prevents a imediate second start cleaning after finish the first one
  if (timer_button_winding_in <= 5 && timer_button_start_cleaning <= 5
      && BW_state_machine_state == 3 && BW_state_machine_timer >= 200) {
    BW_state_machine_state = 0;
  }
  if (timer_button_cable_loose >= TIME_BUTTON_DEBOUNCE) {
    cable_loose = true;
  } else {
    cable_loose = false;
  }
  // reset timer for long press of buttons
  if (timer_button_start_cleaning <= 5) {
    timer_button_long_press = 0;
  }
}

void BugWiper_Task1_fast(void* parameter) {
  const TickType_t taskPeriod = 2;  // 2ms <--> 500Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Encoder_init();

  for (;;) {

    //
    // Do Stuff (needs to take less than 20ms)
    //
    button_debounce();
    BugWiper_set_timer();
    BugWiper_read_motor_current();
    BugWiper_ADC_filter();
    BugWiper_set_motor_power();

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void BugWiper_Task2_slow(void* parameter) {
  const TickType_t taskPeriod = 20;  // 20ms <--> 50Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  //BugWiper_test_Motor();
  for (;;) {

    //
    // Do Stuff (needs to take less than 20ms)
    //
    read_Buttons();
    BugWiper_read_ADCs_slow();
    BugWiper_read_Encoder();
    BugWiper_check_end_reached();
    BugWiper_state_machine();
    BugWiper_LED_blinking();

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void BugWiper_init(void) {
  DEBUG_INFO("Init BugWiper:");
  pinMode(SW_CABLE_LOOSE_PIN, INPUT_PULLUP);
  pinMode(SAFETY_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_WINDING_IN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CLEANING_PIN, INPUT_PULLUP);
  pinMode(RGB_LED_PIN, OUTPUT);
  //analogSetPinAttenuation(MOTOR_CURRENT_SENSE_PIN, ADC_6db);
  digitalWrite(RGB_LED_PIN, 0);
  BugWiper_ADC_filter_init();

  //Encoder_init();
#ifdef BTN9960_CONTROLLER
  btn_motor_control.begin();
  btn_motor_control.set_slew_rate(SLEW_RATE_LEVEL_5);
  HalfBridge_1.set_dk(MOTOR_HB1_DK);
  HalfBridge_2.set_dk(MOTOR_HB2_DK);
  //analogSetPinAttenuation(MOTOR_IS1_PIN, ADC_6db);
  //analogSetPinAttenuation(MOTOR_IS2_PIN, ADC_6db);
#endif

#ifdef BTS7960B_CONTROLLER
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, 1);
  digitalWrite(MOTOR_IN2_PIN, 1);
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
#endif  // BTS7960B_CONTROLLER
  xTaskCreate(BugWiper_Task1_fast, "BW_T1_fast", 1024 * 2, NULL, 3, NULL);
  xTaskCreate(BugWiper_Task2_slow, "BW_T2_alow", 1024 * 8, NULL, 3, NULL);
}