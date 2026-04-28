#include "esp32-hal-gpio.h"
#include "esp32-hal-adc.h"
#include "esp32-hal-ledc.h"
#include <sys/_stdint.h>
#include <Arduino.h>
#include "my_debug.h"
#include "BugWiper.h"
#include "btn99x0_motor_control.hpp"
#include "sd_logger.h"

#define ADC_FILTER_SIZE 32

// String representation of BW_MODE for logging and debugging
static const char* BW_MODE_STR[BW_MODE_COUNT] = {
  "IDLE",
  "REFERENCE_IN",
  "START_CLEAN_OUT",
  "CLEANING",
  "DECEL_LOOSE",
  "WIGGLE_LOOSE",
  "RESTART_AFTER_LOOSE",
  "DECEL_END",
  "WINDING_IN",
  "GROUND_OUT",
  "FINISHED",
  "EMERGENCY_IN",
  "STOP",
  "ERROR"
};

const char* bwModeToString(BW_MODE mode)
{
  if (mode >= BW_MODE_COUNT) {
    return "INVALID_MODE";
  }
  return BW_MODE_STR[mode];
}

// Mode configuration table indexed by BW_MODE
const BW_ModeConfig bw_modeConfig[BW_MODE_COUNT] = {

  /* ------------------------------------------------------------
   * M_IDLE
   * ------------------------------------------------------------ */
  {
    // Motor behavior
    .motorCmd = {
      .dir         = STOP,
      .startPower  = 0,
      .targetPower = 0,
      .rampTime    = 0
    },

    // Timing
    .minTime = 0,
    .maxTime = 0,

    // LED
    .ledColor     = BLACK,
    .ledBlinkTime = 0,

    // Behavior flags
    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false
  },

  /* ------------------------------------------------------------
   * M_REFERENCE_IN
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = IN,
      .startPower  = 120,
      .targetPower = 200,
      .rampTime    = 6
    },

    .minTime = 0,
    .maxTime = 5000,

    .ledColor     = CYAN,
    .ledBlinkTime = 5,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    
    .defaultNext = M_START_CLEAN_OUT
  },

  /* ------------------------------------------------------------
   * M_START_CLEAN_OUT
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 40,
      .targetPower = 120,
      .rampTime    = 6
    },

    .minTime = 0,
    .maxTime = 3000,

    .ledColor     = GREEN,
    .ledBlinkTime = LED_TIME_CLEANING,

    .allowLooseDetect = true,
    .allowWiggle      = true,
    .ignoreSafety     = false,

    .defaultNext = M_CLEANING
  },

  /* ------------------------------------------------------------
   * M_CLEANING
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 50,
      .targetPower = 200,
      .rampTime    = 6
    },

    .minTime = TIME_MIN_CLEANING,
    .maxTime = TIME_MAX_CLEANING,

    .ledColor     = GREEN,
    .ledBlinkTime = LED_TIME_CLEANING,

    .allowLooseDetect = true,
    .allowWiggle      = true,
    .ignoreSafety     = false,

    .defaultNext = M_DECEL_END
  },

  /* ------------------------------------------------------------
   * M_DECEL_LOOSE
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = STOP,
      .startPower  = 0,
      .targetPower = 0,
      .rampTime    = 8
    },

    .minTime = 0,
    .maxTime = 2000,

    .ledColor     = ORANGE,
    .ledBlinkTime = 150,

    .allowLooseDetect = false,
    .allowWiggle      = true,
    .ignoreSafety     = false,

    .defaultNext = M_RESTART_AFTER_LOOSE
  },

  /* ------------------------------------------------------------
   * M_WIGGLE_LOOSE
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = IN,
      .startPower  = 50,
      .targetPower = 100,
      .rampTime    = 2
    },

    .minTime = 0,
    .maxTime = 1000,

    .ledColor     = ORANGE,
    .ledBlinkTime = 120,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    .defaultNext = M_RESTART_AFTER_LOOSE
  },

  /* ------------------------------------------------------------
   * M_RESTART_AFTER_LOOSE
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 50,
      .targetPower = 200,
      .rampTime    = 6
    },

    .minTime = 0,
    .maxTime = 3000,

    .ledColor     = GREEN,
    .ledBlinkTime = LED_TIME_CLEANING,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    .defaultNext = M_CLEANING
  },

  /* ------------------------------------------------------------
   * M_DECEL_END
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 200,
      .targetPower = 20,
      .rampTime    = 3
    },

    .minTime = 0,
    .maxTime = 500,

    .ledColor     = BLUE,
    .ledBlinkTime = 0,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    .defaultNext = M_WINDING_IN
  },

  /* ------------------------------------------------------------
   * M_WINDING_IN
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = IN,
      .startPower  = 10,
      .targetPower = 255,
      .rampTime    = 4
    },

    .minTime = 0,
    .maxTime = 5000,

    .ledColor     = BLUE,
    .ledBlinkTime = 250,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    .defaultNext = M_FINISHED
  },

  /* ------------------------------------------------------------
  * M_GROUND_OUT
  * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 10,
      .targetPower = 80,
      .rampTime    = 15
    },

    .minTime = 0,
    .maxTime = 3000,        // short timeout, no long running

    .ledColor     = ORANGE,
    .ledBlinkTime = 100,

    .allowLooseDetect = true,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    .defaultNext = M_FINISHED
  },

  /* ------------------------------------------------------------
   * M_FINISHED
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = STOP,
      .startPower  = 0,
      .targetPower = 0,
      .rampTime    = 0
    },

    .minTime = 0,
    .maxTime = 0,

    .ledColor     = GREEN,
    .ledBlinkTime = 0,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    .defaultNext = M_IDLE
  },

  /* ------------------------------------------------------------
   * M_EMERGENCY_IN
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = IN,
      .startPower  = 50,
      .targetPower = 250,
      .rampTime    = 3
    },

    .minTime = 0,
    .maxTime = 0,     // No timeout

    .ledColor     = ORANGE,
    .ledBlinkTime = 100,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = true,

    .defaultNext = M_IDLE
  },

  /* ------------------------------------------------------------
   * M_STOP
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = STOP,
      .startPower  = 0,
      .targetPower = 0,
      .rampTime    = 0
    },

    .minTime = 0,
    .maxTime = 2000,

    .ledColor     = ORANGE,
    .ledBlinkTime = 0,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    .defaultNext = M_FINISHED
  },

  /* ------------------------------------------------------------
   * M_ERROR
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = STOP,
      .startPower  = 0,
      .targetPower = 0,
      .rampTime    = 0
    },

    .minTime = 0,
    .maxTime = 2000,

    .ledColor     = RED,
    .ledBlinkTime = 500,

    .allowLooseDetect = false,
    .allowWiggle      = false,
    .ignoreSafety     = false,

    .defaultNext = M_IDLE
  }
};


BW_MODE bw_currentMode = M_IDLE;
BW_SubState bw_subState = SUB_INIT;

uint32_t bw_modeStartTime = 0;

static UserCommand lastUserCommand = CMD_NONE;

uint32_t BW_ADC_current_sense;
uint32_t ADC_current_filter_sum;
uint32_t ADC_current_old_values[ADC_FILTER_SIZE];
int8_t ADC_filter_counter = 0;
uint32_t BW_ADC_current_filtered;

double BW_ADC_current_mA;
uint16_t BW_ADC_current_counter;
volatile double BW_ADC_current_mA_filtered;
float BW_ADC_T_ntc_degree;
float BW_ADC_V_Bat;
volatile double BW_ADC_btn_hb1;
volatile double BW_ADC_btn_hb2;

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

volatile uint16_t timer_LED;
uint16_t LED_time;
volatile uint8_t timer_motor_power;

bool motor_inverted;


const PositionConfig positionConfig = {
  .startSlowOut  = POSITION_STARTING,
  .slowZoneStart = POSITION_SLOW_WINGTIP,
  .wingTip       = POSITION_WINGTIP,
  .groundOutMax  = 800
};


static direction motor_direction;
static uint8_t motor_power = 0;        // current power
static uint8_t motor_power_dest = 0;   // target power
static uint8_t time_pwm_ramp = 0;

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
  3.1,   // VOltage Range
  8191   // ADC Steps
};

MotorState bw_motorState;


DCShield shield(hb1_io_pins, hb2_io_pins, hw_conf);
MotorControl btn_motor_control(shield);
HalfBridge HalfBridge_1 = shield.get_half_bridge(DCShield::HALF_BRIDGE_1);
HalfBridge HalfBridge_2 = shield.get_half_bridge(DCShield::HALF_BRIDGE_2);

void Encoder_init(void) {
  BW_motor_encoder.attachHalfQuad(MOTOR_ENCODER_1_PIN, MOTOR_ENCODER_2_PIN);
  BW_motor_encoder.setCount(0);
}

void BugWiper_rgbLedWrite(struct RGB_COLOUR colour) {
  rgbLedWrite(RGB_LED_PIN, colour.g, colour.r, colour.b);
}


void setLED(const RGB_COLOUR& color, uint16_t blinkTime)
{
  BugWiper_rgbLedWrite(color);
  LED_time = blinkTime;
}

void BugWiper_log(void){
  unsigned long t = millis();
  DEBUG_INFO("Time:" + String(t));
  DEBUG_INFO("ADC_Current:" + String(BW_ADC_current_mA_filtered) + " HB1:" + String(BW_ADC_btn_hb1) + " HB2:" + String(BW_ADC_btn_hb2));
  DEBUG_INFO("Encoder_count:" + String((int32_t)motor_enc_count) + " Power:" + String(motor_power));
  DEBUG_INFO("Position:" + String(BW_position) + " Speed:" + String(BW_speed));
  DEBUG_WARNING(String("State: ") + bwModeToString(bw_currentMode));
  DEBUG_INFO("ADC_VBat:" + String(BW_ADC_V_Bat) + " NTC:" + String(BW_ADC_T_ntc_degree));
  sdLoggerLog(t, bw_currentMode, BW_position, BW_speed, BW_ADC_current_mA_filtered, BW_ADC_V_Bat);
}

void BugWiper_log_event(void){
  unsigned long t = millis();
  DEBUG_WARNING("Time:" + String(t));
  DEBUG_WARNING("ADC_Current:" + String(BW_ADC_current_mA_filtered) + " HB1:" + String(BW_ADC_btn_hb1) + " HB2:" + String(BW_ADC_btn_hb2));
  DEBUG_WARNING("Encoder_count:" + String((int32_t)motor_enc_count) + " Power:" + String(motor_power));
  DEBUG_WARNING("Position:" + String(BW_position) + " Speed:" + String(BW_speed));
  DEBUG_WARNING(String("State: ") + bwModeToString(bw_currentMode));
  DEBUG_WARNING("ADC_VBat:" + String(BW_ADC_V_Bat) + " NTC:" + String(BW_ADC_T_ntc_degree));
  sdLoggerLog(t, bw_currentMode, BW_position, BW_speed, BW_ADC_current_mA_filtered, BW_ADC_V_Bat);
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

void bw_motorInit(const MotorCommand& cmd)
{
  bw_motorState.dir         = cmd.dir;
  bw_motorState.power       = cmd.startPower;
  bw_motorState.targetPower = cmd.targetPower;
  bw_motorState.rampTime    = cmd.rampTime;
  bw_motorState.rampTimer   = 0;
}

void bw_set_motor_power(void) {
  if (bw_motorState.rampTimer >= bw_motorState.rampTime) {
    if (bw_motorState.power < bw_motorState.targetPower) {
      bw_motorState.power++;
    } else if (bw_motorState.power > bw_motorState.targetPower) {
      bw_motorState.power--;
    }
    bw_motorState.rampTimer = 0;
  }
#ifdef BTN9960_CONTROLLER
  switch (motor_direction) {
    case OUT:
      btn_motor_control.set_speed(-bw_motorState.power);
      break;
    case IN:
      btn_motor_control.set_speed(bw_motorState.power);
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
  switch (motor_direction) {
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

    case Freewheeling:
      digitalWrite(MOTOR_IN2_PIN, 0);
      digitalWrite(MOTOR_IN1_PIN, 0);
      break;
  }
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Code for version 3.x
  ledcWrite(motor_pwm_pin, bw_motorState.power);
#else
  // Code for version 2.x
  ledcWrite(motor_pwm_channel, bw_motorState.power);
#endif
#endif
}

void BugWiper_LED_blinking(void) {
  if (timer_LED >= LED_time) {
    //digitalWrite(RGB_LED_PIN, !digitalRead(RGB_LED_PIN)); Fixme
    timer_LED = 0;
  }
}

bool stateTimedOut(uint32_t maxTime)
{
  if (maxTime == 0) return false;
  return (millis() - bw_modeStartTime) > maxTime;
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
  adc_temp = (uint16_t)BW_ADC_T_ntc_degree;
  BW_ADC_T_ntc_degree = ((float)adc_temp * (float)adc_temp * (float)adc_temp * (-2.87638e-9)) + ((float)adc_temp * (float)adc_temp * (2.01243e-5)) + ((-0.0702) * (float)adc_temp) + 109.013;
  BW_ADC_V_Bat = analogReadMilliVolts(ADC_VBat_PIN) * 0.0081;
}

bool bw_check_end_reached(void) {
  // Current-based detection
  if (BW_ADC_current_mA_filtered >= BW_STOP_CURRENT) {
    BW_ADC_current_counter++;
    if (BW_ADC_current_counter > BW_STOP_CURRENT_COUNTS) {
      DEBUG_INFO("Finished: current:" + String(BW_ADC_current_mA) + " above " + String((float)BW_STOP_CURRENT));
      BugWiper_log_event();
      return true;
    }
  } else {
    if (BW_ADC_current_counter > 0) {
      BW_ADC_current_counter--;
    }
  }
  
  // Speed-based detection
  if (abs(BW_speed) < BW_STOP_SPEED && BW_state_machine_timer > TIME_MIN_CLEANING) {
    BW_speed_counter++;
    if (BW_speed_counter >= BW_STOP_SPEED_COUNTS) {
      DEBUG_INFO("Finished: Speed:" + String(abs(BW_speed)) + " below " + String((float)BW_STOP_SPEED));
      BugWiper_log_event();
      return true;
    }
  } else {
    if (BW_speed_counter > 1) {
      BW_speed_counter--;
    }
  }
  return false;
}

bool motorSlowedDown(void) {
  if ( BW_speed < BW_STOP_SPEED ) {
    BW_speed_counter++;
    if (BW_speed_counter >= BW_STOP_SPEED_COUNTS) {
      DEBUG_INFO("Decel End Finished: Speed:" + String(BW_speed) + " below " + String((float)BW_STOP_SPEED));
      BugWiper_log_event();
      return true;
    }
  } else {
    if (BW_speed_counter > 1) {
      BW_speed_counter--;
    }
  }
}

bool BugWiper_safety_protection(void) {
  // Undervoltage protection
  if (BW_ADC_V_Bat <= BW_STOP_V_BAT) {
    DEBUG_ERROR("Under Voltage: V BAT:" + String(BW_ADC_V_Bat) + " below " + String((float)BW_STOP_V_BAT) + "V");
    BugWiper_log_event();
    return true;
  }

  //Temperature protection
  if (BW_ADC_T_ntc_degree > BW_STOP_T_MAX) {
    DEBUG_ERROR("Over Temperature: T NTC:" + String(BW_ADC_T_ntc_degree) + "above " + String((float)BW_STOP_T_MAX) + "DEG");
    BugWiper_log_event();
    return true;
  }

  return false;
}

void BugWiper_set_timer(void) {
  BW_state_machine_timer++;
  BW_state_machine_timer_2++;
  timer_motor_power++;
  timer_LED++;
}

void BugWiper_read_Encoder(void) {
  uint32_t BW_enc_count_old = motor_enc_count;
  motor_enc_count = BW_motor_encoder.getCount();
  BW_position = int32_t((float)motor_enc_count * SPOOL_CIRCUMFERENCE / (CPR_Encoder * GEAR_RATIO));
  BW_speed = motor_enc_count - BW_enc_count_old;
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

bool eventStopRequested(void)
{
  // Stop cleaning by pressing winding-in button
  if (lastUserCommand == CMD_CLEANING &&
      timer_button_winding_in >= TIME_BUTTON_DEBOUNCE) {

    DEBUG_WARNING("Stop requested: cleaning");
    return true;
  }

  // Stop winding-in by pressing cleaning button
  if (lastUserCommand == CMD_WINDING_IN &&
      timer_button_start_cleaning >= TIME_BUTTON_DEBOUNCE) {

    DEBUG_WARNING("Stop requested: winding in");
    return true;
  }

  return false;
}

bool buttonOutPressed(void) {
  if (timer_button_winding_in >= TIME_BUTTON_DEBOUNCE) {
    return true;
  }
  return false;
}

bool buttonInPressed(void) {
  if (timer_button_start_cleaning >= TIME_BUTTON_DEBOUNCE) {
    return true;
  }
  return false;
}

bool eventCableLoose(void) {
  if (timer_button_cable_loose >= TIME_BUTTON_DEBOUNCE) {
    return true;
  } else {
    return false;
  }
}

bool groundModeEnabled(void) {
  return false;
}

void changeMode(BW_MODE newMode)
{
  DEBUG_INFO("FSM transition: %s -> %s",
             bwModeToString(bw_currentMode),
             bwModeToString(newMode));

  bw_currentMode = newMode;
  const BW_ModeConfig& cfg = bw_modeConfig[newMode];

  bw_modeStartTime = millis();
  bw_subState = SUB_INIT;

  // Apply LED for this mode
  setLED(cfg.ledColor, cfg.ledBlinkTime);
}


// Handle global transitions with highest priority
bool handleGlobalTransitions() {
  // User stop request (opposite button)
  if (eventStopRequested()) {
    changeMode(M_STOP);
    return true;
  }

  // System error conditions
  if (BugWiper_safety_protection()) {
    changeMode(M_ERROR);
    return true;
  }

  return false;  // No global transition taken
}

void stateIdle(const BW_ModeConfig& cfg) {
  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      lastUserCommand = CMD_NONE;
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      // Waiting for user input
      if (groundModeEnabled() && buttonOutPressed()) {
        lastUserCommand = CMD_CLEANING;   // ground out is still an "out" operation
        changeMode(M_GROUND_OUT);
      }
      else if (buttonInPressed()) {
        lastUserCommand = CMD_WINDING_IN;
        changeMode(M_WINDING_IN);
      }
      else if (buttonOutPressed()) {
        lastUserCommand = CMD_CLEANING;
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateReferenceIn(const BW_ModeConfig& cfg) {

  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING: {
      bool endReached = bw_check_end_reached();
      bool minTimeElapsed = (millis() - bw_modeStartTime) >= cfg.minTime;

      // Transition condition: BOTH must be true
      if (endReached && minTimeElapsed) {
        bw_subState = SUB_DONE;
      }
      break;
    }

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateStartCleanOut(const BW_ModeConfig& cfg) {

  if (cfg.allowLooseDetect && eventCableLoose()) {
    changeMode(M_DECEL_LOOSE);
    return;
  }

  if (stateTimedOut(cfg.maxTime)) {
    changeMode(M_ERROR);
    return;
  }

  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (BW_position >= positionConfig.slowZoneStart) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateCleaning(const BW_ModeConfig& cfg) {
  if (stateTimedOut(cfg.maxTime)) {
    changeMode(M_ERROR);
    return;
  }

  if (cfg.allowLooseDetect && eventCableLoose()) {
    changeMode(M_DECEL_LOOSE);
    return;
  }

  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (BW_position >= positionConfig.startSlowOut) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}


void stateDecelLoose(const BW_ModeConfig& cfg) {
  if (BW_position >= positionConfig.slowZoneStart) {
    changeMode(M_WIGGLE_LOOSE);
    return;
  }

  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (!eventCableLoose()){
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateWiggleLoose(const BW_ModeConfig& cfg) {
}

void stateRestartAfterLoose(const BW_ModeConfig& cfg) {
  if (cfg.allowLooseDetect && eventCableLoose()) {
    changeMode(M_DECEL_LOOSE);
    return;
  }

  if (stateTimedOut(cfg.maxTime)) {
    changeMode(M_ERROR);
    return;
  }

  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (BW_position >= positionConfig.slowZoneStart) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateDecelEnd(const BW_ModeConfig& cfg) {
    switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
     if (motorSlowedDown()) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateWindingIn(const BW_ModeConfig& cfg) {
  if (stateTimedOut(cfg.maxTime)) {
    changeMode(M_ERROR);
    return;
  }

  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if(bw_check_end_reached()) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateGroundOut(const BW_ModeConfig& cfg) {
  // Ground / maintenance outward movement only

  if (stateTimedOut(cfg.maxTime)) {
    changeMode(M_ERROR);
    return;
  }

  if (stateTimedOut(cfg.maxTime)) {
    changeMode(M_ERROR);
    return;
  }

  if (cfg.allowLooseDetect && eventCableLoose()) {
    changeMode(M_DECEL_LOOSE);
    return;
  }

  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (BW_position >= positionConfig.groundOutMax) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateFinished(const BW_ModeConfig& cfg) {
  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (stateTimedOut(cfg.maxTime)) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateEmergencyIn(const BW_ModeConfig& cfg) {
  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (stateTimedOut(cfg.maxTime)) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateStop(const BW_ModeConfig& cfg) {
  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (stateTimedOut(cfg.maxTime)) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void stateError(const BW_ModeConfig& cfg) {
  switch (bw_subState) {

    case SUB_INIT:
      bw_motorInit(cfg.motorCmd);
      bw_subState = SUB_RUNNING;
      break;

    case SUB_RUNNING:
      if (stateTimedOut(cfg.maxTime)) {
        bw_subState = SUB_DONE;
      }
      break;

    case SUB_DONE:
      changeMode(cfg.defaultNext);
      break;
  }
}

void BugWiper_processFSM()
{
  // ------------------------------------------------------------
  // 1. Global priority transitions
  // Emergency, Stop and Error have highest priority
  // ------------------------------------------------------------
  if (handleGlobalTransitions()) {
    return;
  }

  // Fetch configuration for current mode
  const BW_ModeConfig& cfg = bw_modeConfig[bw_currentMode];

  // ------------------------------------------------------------
  // 2. State-specific logic
  // ------------------------------------------------------------

  switch (bw_currentMode)
  {
    case M_IDLE:                  stateIdle(cfg);               break;
    case M_REFERENCE_IN:          stateReferenceIn(cfg);        break;
    case M_START_CLEAN_OUT:       stateStartCleanOut(cfg);      break;
    case M_CLEANING:              stateCleaning(cfg);           break;
    case M_DECEL_LOOSE:           stateDecelLoose(cfg);         break;
    case M_WIGGLE_LOOSE:          stateWiggleLoose(cfg);        break;
    case M_RESTART_AFTER_LOOSE:   stateRestartAfterLoose(cfg);  break;
    case M_DECEL_END:             stateDecelEnd(cfg);           break;
    case M_WINDING_IN:            stateWindingIn(cfg);          break;
    case M_GROUND_OUT:            stateGroundOut(cfg);          break;
    case M_FINISHED:              stateFinished(cfg);              break;
    case M_EMERGENCY_IN:          stateEmergencyIn(cfg);        break;
    case M_STOP:                  stateStop(cfg);                  break;
    case M_ERROR:                 stateError(cfg);                 break;
    default:                      changeMode(M_ERROR);          break;
  }

}


void BugWiper_Task1_fast(void* parameter) {
  const TickType_t taskPeriod = 2;  // 2ms <--> 500Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Encoder_init();

  for (;;) {
    button_debounce();
    BugWiper_set_timer();
    BugWiper_read_motor_current();
    BugWiper_ADC_filter();
    bw_set_motor_power();
    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void BugWiper_Task2_slow(void* parameter) {
  const TickType_t taskPeriod = 20;  // 20ms <--> 50Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  //BugWiper_test_Motor();
  for (;;) {
    BugWiper_read_ADCs_slow();
    BugWiper_read_Encoder();
    BugWiper_processFSM();
    BugWiper_LED_blinking();
    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void BugWiper_rgbLed_init(void) {
  pinMode(RGB_LED_PIN, OUTPUT);
  digitalWrite(RGB_LED_PIN, 0);
}

void BugWiper_init(void) {
  DEBUG_INFO("Init BugWiper:");
  pinMode(SW_CABLE_LOOSE_PIN, INPUT_PULLUP);
  pinMode(SAFETY_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_WINDING_IN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CLEANING_PIN, INPUT_PULLUP);
  //analogSetPinAttenuation(MOTOR_CURRENT_SENSE_PIN, ADC_6db);
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