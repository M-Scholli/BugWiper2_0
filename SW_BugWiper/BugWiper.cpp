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

// Task periods in ms
#define BW_TASK_FAST_MS  2
#define BW_TASK_SLOW_MS  20

// Stop function thresholds
#define BW_STOP_CURRENT 6500
#define BW_STOP_CURRENT_COUNTS 5
#define BW_STOP_SPEED 1
#define BW_STOP_SPEED_COUNTS 20
#define BW_STOP_V_BAT 8.0
#define BW_STOP_T_MAX 70

// Sensor filter parameters (more stable)
#define BW_SENSOR_FILTER_THRESHOLD  30
// Button filter parameters (fast reaction)
#define BW_BTN_FILTER_THRESHOLD  10
#define BW_BTN_DEBOUNCE_TICKS  3    // fast task ticks
#define BW_BTN_HOLD_TICKS     100  // fast task ticks

// String representation of BW_MODE for logging and debugging
static const char* BW_MODE_STR[BW_MODE_COUNT] = {
  "IDLE",
  "REFERENCE_IN",
  "START_CLEAN_OUT",
  "CLEANING",
  "DECEL_LOOSE",
  "WIGGLE_IN",
  "WIGGLE_OUT",
  "DECEL_END",
  "WINDING_IN",
  "WINDING_IN_DECEL",
  "GROUND_OUT",
  "DECEL_LOOSE_GROUND",
  "FINISHED",
  "EMERGENCY_IN",
  "STOP",
  "ERROR"
};

const char* bw_ModeToString(BW_MODE mode)
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

    // LED behavior
    .ledCmd = {
      .color     = BLACK,
      .blinkTime = 0
    },

    // Timing
    .minTime = 0,
    .maxTime = 0,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = false,
    .enableMaxTimeCheck    = false,
    .enableStopRequest     = false,
    .enableWingTipCheck    = false,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_IDLE
  },

  /* ------------------------------------------------------------
   * M_REFERENCE_IN
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = IN,
      .startPower  = 80,
      .targetPower = 200,
      .rampTime    = 20
    },

    .ledCmd = {
      .color     = GREEN,
      .blinkTime = 100
    },

    .minTime = 1000,
    .maxTime = 5000,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = false,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_START_CLEAN_OUT
  },

  /* ------------------------------------------------------------
   * M_START_CLEAN_OUT
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 40,
      .targetPower = 150,
      .rampTime    = 15
    },

    .ledCmd = {
      .color     = GREEN,
      .blinkTime = 400
    },

    .minTime = 500,
    .maxTime = 5000,

    // Global transition flags
    .enableEndCheckCurrent = true,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = true,
    .requireMinTimeForEndCheck = false,  // minTime > 0

    .defaultNext = M_CLEANING
  },

  /* ------------------------------------------------------------
   * M_CLEANING
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 0,
      .targetPower = 255,
      .rampTime    = 7
    },

    .ledCmd = {
      .color     = GREEN,
      .blinkTime = 400
    },

    .minTime = 250,
    .maxTime = 15000,

    // Global transition flags
    .enableEndCheckCurrent = true,
    .enableEndCheckSpeed   = true,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = true,
    .requireMinTimeForEndCheck = true,

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
      .rampTime    = 0
    },

    .ledCmd = {
      .color     = ORANGE,
      .blinkTime = 400
    },

    .minTime = 0,
    .maxTime = 10000,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = true,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_START_CLEAN_OUT
  },

  /* ------------------------------------------------------------
   * M_WIGGLE_IN
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = IN,
      .startPower  = 100,
      .targetPower = 150,
      .rampTime    = 4
    },

    .ledCmd = {
      .color     = ORANGE,
      .blinkTime = 400
    },

    .minTime = 0,
    .maxTime = 10000,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = true,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_WIGGLE_OUT
  },

  /* ------------------------------------------------------------
   * M_WIGGLE_OUT
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 35,
      .targetPower = 100,
      .rampTime    = 10
    },

    .ledCmd = {
      .color     = ORANGE,
      .blinkTime = 400
    },

    .minTime = 0,
    .maxTime = 10000,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = true,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_CLEANING
  },

  /* ------------------------------------------------------------
   * M_DECEL_END
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 0,
      .targetPower = 0,
      .rampTime    = 4
    },

    .ledCmd = {
      .color     = GREEN,
      .blinkTime = 0
    },

    .minTime = 0,
    .maxTime = 1500,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = true,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_WINDING_IN
  },

  /* ------------------------------------------------------------
   * M_WINDING_IN
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = IN,
      .startPower  = 50,
      .targetPower = 255,
      .rampTime    = 4
    },

    .ledCmd = {
      .color     = BLUE,
      .blinkTime = 300
    },

    .minTime = 1500,
    .maxTime = 20000,

    // Global transition flags
    .enableEndCheckCurrent = true,
    .enableEndCheckSpeed   = true,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = false,
    .requireMinTimeForEndCheck = true,

    .defaultNext = M_FINISHED
  },

  /* ------------------------------------------------------------
   * M_WINDING_IN_DECEL
   * ------------------------------------------------------------
   */
  {
    .motorCmd = {
      .dir         = IN,
      .startPower  = 0,
      .targetPower = 180,
      .rampTime    = 3
    },

    .ledCmd = {
      .color     = BLUE,
      .blinkTime = 150
    },

    .minTime = 500,
    .maxTime = 7000,

    // Global transition flags
    .enableEndCheckCurrent = true,
    .enableEndCheckSpeed   = true,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = false,
    .requireMinTimeForEndCheck = true,

    .defaultNext = M_FINISHED
  },

  /* ------------------------------------------------------------
  * M_GROUND_OUT
  * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = OUT,
      .startPower  = 40,
      .targetPower = 100,
      .rampTime    = 20
    },

    .ledCmd = {
      .color     = CYAN,
      .blinkTime = 150
    },

    .minTime = 0,
    .maxTime = 3000,        // short timeout, no long running

    // Global transition flags
    .enableEndCheckCurrent = true,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = true,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_FINISHED
  },

  /* ------------------------------------------------------------
   * M_DECEL_LOOSE_GROUND
   * ------------------------------------------------------------ */
  {
    .motorCmd = {
      .dir         = STOP,
      .startPower  = 0,
      .targetPower = 0,
      .rampTime    = 0
    },

    .ledCmd = {
      .color     = ORANGE,
      .blinkTime = 150
    },

    .minTime = 0,
    .maxTime = 10000,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = true,
    .enableMaxTimeCheck    = true,
    .enableStopRequest     = true,
    .enableWingTipCheck    = true,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_GROUND_OUT
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

    .ledCmd = {
      .color     = GREEN,
      .blinkTime = 0
    },

    .minTime = 0,
    .maxTime = 0,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = false,
    .enableMaxTimeCheck    = false,
    .enableStopRequest     = false,
    .enableWingTipCheck    = false,
    .requireMinTimeForEndCheck = false,

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
      .rampTime    = 10
    },

    .ledCmd = {
      .color     = ORANGE,
      .blinkTime = 150
    },

    .minTime = 0,
    .maxTime = 0,     // No timeout

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = false,  // Emergency mode
    .enableMaxTimeCheck    = false,   // maxTime = 0
    .enableStopRequest     = true,
    .enableWingTipCheck    = false,
    .requireMinTimeForEndCheck = false,

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

    .ledCmd = {
      .color     = RED,
      .blinkTime = 0
    },

    .minTime = 0,
    .maxTime = 2000,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = false,
    .enableMaxTimeCheck    = false,
    .enableStopRequest     = false,
    .enableWingTipCheck    = false,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_IDLE
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

    .ledCmd = {
      .color     = RED,
      .blinkTime = 150
    },

    .minTime = 0,
    .maxTime = 2000,

    // Global transition flags
    .enableEndCheckCurrent = false,
    .enableEndCheckSpeed   = false,
    .enableSafetyProtection = false,
    .enableMaxTimeCheck    = false,
    .enableStopRequest     = false,
    .enableWingTipCheck    = false,
    .requireMinTimeForEndCheck = false,

    .defaultNext = M_IDLE
  }
};

BW_MODE bw_currentMode = M_IDLE;
uint32_t bw_modeStartTime = 0;

static UserCommand lastUserCommand = CMD_NONE;

ADCFilter adc_current_filter;
ADCFilter adc_voltage_filter;
ADCFilter adc_tempNTC_filter;

ADCValues adc_values;
StopDetection stop_detection;

BwFilter bw_cableLooseFilter;
BwFilter bw_groundSwitchFilter;

BwFilter bw_btnInFilter;
BwFilter bw_btnOutFilter;
ButtonRuntime bw_btnIn;
ButtonRuntime bw_btnOut;

#ifdef BTS7960B_CONTROLLER
int motor_pwm_channel;
gpio_num_t motor_pwm_pin;
#endif

const PositionConfig positionConfig = {
  #ifdef TESTBENCH
  .slowZoneStartOut        = 100,    // Slow start length in mm
  .slowZoneWingTip         = 1400,   // Start slow before wing tip
  .wingTip                 = 1600,   // End of the wing
  #else
  .slowZoneStartOut        = 200,    // Slow start length in mm
  .slowZoneWingTip         = 5200,   // Start slow before wing tip
  .wingTip                 = 5500,   // End of the wing
  #endif
  .groundOutMax            = 1000,    // Maximum extension in ground mode
  .windingInDecelDistance  = 500,    // Distance to start deceleration before fuselage
  .windingInDecelSpeed     = 40,     // Speed threshold to switch to deceleration mode
  .wiggleStraightDistance  = 75      // Distance to check cable tightness during wiggle OUT
};

const WiggleTiming wiggleTimingConfig = {
  .inDuration_ms         = 100,    // Retract (IN) for 1000ms
  .outDuration_ms        = 450,    // Extend (OUT) for 1000ms
  .minRetractTime_ms     = 550,     // Minimum retract time before checking cable on OUT
  .decelLooseToWiggleTime_ms = 150,  // Time after which to switch to wiggle if cable still loose and motor slowed down
  .totalWiggleTimeout_ms = 5000     // Total timeout for entire wiggle process
};

ESP32Encoder bw_motorEncoder;

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
LedState bw_ledState;

DCShield shield(hb1_io_pins, hb2_io_pins, hw_conf);
MotorControl btn_motor_control(shield);
HalfBridge HalfBridge_1 = shield.get_half_bridge(DCShield::HALF_BRIDGE_1);
HalfBridge HalfBridge_2 = shield.get_half_bridge(DCShield::HALF_BRIDGE_2);

void bw_encoderSetZero(void) {
  bw_motorEncoder.setCount(0);
  bw_motorState.encoder_count = bw_motorEncoder.getCount();
  bw_motorState.position_mm = int32_t((float)bw_motorState.encoder_count * SPOOL_CIRCUMFERENCE / (CPR_Encoder * GEAR_RATIO));
  bw_motorState.speed = 0;
}

void bw_encoder_init(void) {
  bw_motorEncoder.attachHalfQuad(MOTOR_ENCODER_1_PIN, MOTOR_ENCODER_2_PIN);
  bw_encoderSetZero();
}

void bw_encoderRead(void) {
  uint32_t BW_enc_count_old = bw_motorState.encoder_count;
  bw_motorState.encoder_count = bw_motorEncoder.getCount();
  bw_motorState.position_mm = int32_t((float)bw_motorState.encoder_count * SPOOL_CIRCUMFERENCE / (CPR_Encoder * GEAR_RATIO));
  bw_motorState.speed = bw_motorState.encoder_count - BW_enc_count_old;
}

void bw_rgbLedWrite(struct RGB_COLOUR colour) {
  rgbLedWrite(RGB_LED_PIN, colour.g, colour.r, colour.b);
}

void bw_btnIn_log(void) {
  if (bw_btnIn.event != CMD_NONE) {
    DEBUG_INFO("BTN IN: " + String(bw_btnIn.state) + " Event: " + String(bw_btnIn.event));
  }
}

void bw_btnOut_log(void) {
  if (bw_btnOut.event != CMD_NONE) {
    DEBUG_INFO("BTN OUT: " + String(bw_btnOut.state) + " Event: " + String(bw_btnOut.event));
  }
}

void bw_btn_log(void) {
  bw_btnIn_log();
  bw_btnOut_log();
}

void bw_log(void){
  unsigned long t = millis();
  DEBUG_INFO("Time:" + String(t)+ "  State: " + bw_ModeToString(bw_currentMode));
  DEBUG_INFO("ADC_Current:" + String(adc_values.current_mA_filtered)+ "  M_Power:" + String(bw_motorState.power));
  DEBUG_INFO("Encoder_count:" + String((int32_t)bw_motorState.encoder_count) + "  Position:" + String(bw_motorState.position_mm) + "  Speed:" + String(bw_motorState.speed));
  DEBUG_INFO("ADC_VBat:" + String(adc_values.BatteryVoltage_filtered) + "  NTC:" + String(adc_values.tempNTC_degree));
  sdLoggerLog(t, bw_currentMode, bw_motorState.position_mm, bw_motorState.speed, adc_values.current_mA_filtered, adc_values.BatteryVoltage_filtered);
}

void bw_log_event(void){
  unsigned long t = millis();
  DEBUG_WARNING("Time:" + String(t)+ "  State: " + bw_ModeToString(bw_currentMode));
  DEBUG_WARNING("ADC_Current:" + String(adc_values.current_mA_filtered)+ "  M_Power:" + String(bw_motorState.power));
  DEBUG_WARNING("Encoder_count:" + String((int32_t)bw_motorState.encoder_count) + "  Position:" + String(bw_motorState.position_mm) + "  Speed:" + String(bw_motorState.speed));
  DEBUG_WARNING("ADC_VBat:" + String(adc_values.BatteryVoltage_filtered) + "  NTC:" + String(adc_values.tempNTC_degree));
  sdLoggerLog(t, bw_currentMode, bw_motorState.position_mm, bw_motorState.speed, adc_values.current_mA_filtered, adc_values.BatteryVoltage_filtered);
}

void bw_test_Motor(void) {
  DEBUG_INFO("Run forward for 2 sec...")
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));
  rgbLedWrite(RGB_LED_PIN, RGB_BRIGHTNESS, 0, 0);  // Red
  btn_motor_control.set_speed(180);
  delay(500);
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));
  delay(500);
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));
  DEBUG_INFO("Load current (A): ");
  DEBUG_INFO(HalfBridge_1.get_load_current_in_amps());
  DEBUG_INFO(HalfBridge_2.get_load_current_in_amps());
  DEBUG_INFO(((float)analogReadMilliVolts(MOTOR_CURRENT_SENSE_PIN) * 0.005));
  delay(1000);
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));

  DEBUG_INFO("Freewheel for 1 sec...");
  rgbLedWrite(RGB_LED_PIN, 0, RGB_BRIGHTNESS, 0);  // Green
  btn_motor_control.freewheel();
  delay(1000);
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));

  DEBUG_INFO("Run backward for 2 sec...");
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));
  rgbLedWrite(RGB_LED_PIN, 0, 0, RGB_BRIGHTNESS);  // Blue
  btn_motor_control.set_speed(-180);
  delay(500);
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));
  delay(500);
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));
  DEBUG_INFO("Load current (A): ");
  DEBUG_INFO(HalfBridge_1.get_load_current_in_amps());
  DEBUG_INFO(HalfBridge_2.get_load_current_in_amps());
  DEBUG_INFO(((float)analogReadMilliVolts(MOTOR_CURRENT_SENSE_PIN) * 0.005));
  delay(1000);
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));

  DEBUG_INFO("Brake for 1 sec...");
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);  // Blue
  btn_motor_control.brake();
  delay(1000);
  DEBUG_INFO("Encoder count = " + String((int32_t)bw_motorEncoder.getCount()));
}

void bw_motorInit(const MotorCommand& cmd)
{
  bw_motorState.dir         = cmd.dir;
  if (cmd.startPower > 0) {
    bw_motorState.power = cmd.startPower;
  }
  bw_motorState.targetPower = cmd.targetPower;
  if (cmd.rampTime == 0) {
    bw_motorState.rampTicks = 0;
  } else {
    bw_motorState.rampTicks =
      (cmd.rampTime + BW_TASK_FAST_MS - 1) / BW_TASK_FAST_MS;  // Convert ms to fast task ticks, round up
  }
  bw_motorState.rampTimer   = 0;
}

void bw_motorUpdate(void) {
 if (bw_motorState.rampTicks == 0) {
    // Immediate target
    bw_motorState.power = bw_motorState.targetPower;
  } else {
    bw_motorState.rampTimer++;

    if (bw_motorState.rampTimer >= bw_motorState.rampTicks) {
      if (bw_motorState.power < bw_motorState.targetPower) {
        bw_motorState.power++;
      } else if (bw_motorState.power > bw_motorState.targetPower) {
        bw_motorState.power--;
      }
      bw_motorState.rampTimer = 0;
    }
  }
  
#ifdef BTN9960_CONTROLLER
  switch (bw_motorState.dir) {
    case OUT:           btn_motor_control.set_speed(-bw_motorState.power); break;
    case IN:            btn_motor_control.set_speed(bw_motorState.power); break;
    case STOP:          btn_motor_control.brake(); break;
    case Freewheeling:  btn_motor_control.freewheel(); break;
    default:            btn_motor_control.brake(); break;
  }
#elif defined(BTS7960B_CONTROLLER)
  switch (bw_motorState.dir) {
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

// Initialize LED for new FSM mode
void bw_ledInit(const LedCommand& cmd)
{
  bw_ledState.color = cmd.color;
  bw_ledState.timer = 0;
  bw_ledState.isOn  = true;

  if (cmd.blinkTime == 0) {
    bw_ledState.blinkTicks = 0;
  } else {
    bw_ledState.blinkTicks =
      (cmd.blinkTime + BW_TASK_SLOW_MS - 1) / BW_TASK_SLOW_MS;
  }

  bw_rgbLedWrite(cmd.color);
}

// Update LED state
// bw_ledUpdate() must be called every BW_TASK_SLOW_MS
void bw_ledUpdate(void)
{
  // No blinking
  if (bw_ledState.blinkTicks == 0) {
    return;
  }

  bw_ledState.timer++;

  if (bw_ledState.timer >= bw_ledState.blinkTicks) {
    bw_ledState.timer = 0;
    bw_ledState.isOn = !bw_ledState.isOn;

    if (bw_ledState.isOn) {
      bw_rgbLedWrite(bw_ledState.color);
    } else {
      bw_rgbLedWrite(BLACK);
    }
  }
}

bool stateTimedOut(uint32_t maxTime)
{
  if (maxTime == 0) return false;
  return (millis() - bw_modeStartTime) > maxTime;
}

void bw_read_motor_current(void) {
  uint32_t current_mV = analogReadMilliVolts(MOTOR_CURRENT_SENSE_PIN);
  
  // Use for filter
  adc_current_filter.filter_sum -= adc_current_filter.old_values[adc_current_filter.counter];
  adc_current_filter.old_values[adc_current_filter.counter] = current_mV;
  adc_current_filter.filter_sum += adc_current_filter.old_values[adc_current_filter.counter];

  adc_values.current_mA_filtered = (adc_current_filter.filter_sum / ADC_FILTER_SIZE) * CURRENT_CAL_FACTOR / 10;
  
  adc_current_filter.counter++;
  if (adc_current_filter.counter >= ADC_FILTER_SIZE) {
    adc_current_filter.counter = 0;
  }
}

void bw_ADC_filter_init(void) {
  uint32_t initial_current_mV = analogReadMilliVolts(MOTOR_CURRENT_SENSE_PIN);
  uint32_t initial_voltage_mV = analogReadMilliVolts(ADC_VBat_PIN);
  uint32_t initial_tempNTC_mV = analogReadMilliVolts(ADC_NTC_PIN);

  adc_current_filter.filter_sum = 0;
  adc_voltage_filter.filter_sum = 0;
  adc_tempNTC_filter.filter_sum = 0;

  adc_values.current_mA_filtered = initial_current_mV;
  adc_values.BatteryVoltage_filtered = initial_voltage_mV;
  adc_values.tempNTC_raw_filtered = initial_tempNTC_mV;

  for (uint8_t j = 0; j < ADC_FILTER_SIZE; j++) {
    adc_current_filter.old_values[j] = initial_current_mV;
    adc_voltage_filter.old_values[j] = initial_voltage_mV;
    adc_tempNTC_filter.old_values[j] = initial_tempNTC_mV;

    adc_current_filter.filter_sum += initial_current_mV;
    adc_voltage_filter.filter_sum += initial_voltage_mV;
    adc_tempNTC_filter.filter_sum += initial_tempNTC_mV;
  }
}

void bw_adcUpdateRotatingMilliVolts(void)
{
  static uint8_t ch = 0;
  uint32_t v;

  switch (ch) {
    case 0:
      v = analogReadMilliVolts(ADC_VBat_PIN);
      if (v != 0) {        
        // Use for filter
        adc_voltage_filter.filter_sum -= adc_voltage_filter.old_values[adc_voltage_filter.counter];
        adc_voltage_filter.old_values[adc_voltage_filter.counter] = v;
        adc_voltage_filter.filter_sum += adc_voltage_filter.old_values[adc_voltage_filter.counter];

        adc_values.BatteryVoltage_filtered = (float)(adc_voltage_filter.filter_sum / ADC_FILTER_SIZE)* 0.0081;
        
        adc_voltage_filter.counter++;
        if (adc_voltage_filter.counter >= ADC_FILTER_SIZE) {
          adc_voltage_filter.counter = 0;
        }
      }
      ch = 1;
      break;

    case 1:
      v = analogReadMilliVolts(ADC_NTC_PIN);
      if (v != 0) {
        // Use for filter
        adc_tempNTC_filter.filter_sum -= adc_tempNTC_filter.old_values[adc_tempNTC_filter.counter];
        adc_tempNTC_filter.old_values[adc_tempNTC_filter.counter] = v;
        adc_tempNTC_filter.filter_sum += adc_tempNTC_filter.old_values[adc_tempNTC_filter.counter];

        adc_values.tempNTC_raw_filtered = (float)(adc_tempNTC_filter.filter_sum / ADC_FILTER_SIZE);
        
        adc_tempNTC_filter.counter++;
        if (adc_tempNTC_filter.counter >= ADC_FILTER_SIZE) {
          adc_tempNTC_filter.counter = 0;
        }
      }
      ch = 0;
      break;
  }
}


void bw_convert_ADCs_slow(void) {
  uint16_t adc_temp;
  adc_values.tempNTC_degree = adc_values.tempNTC_raw_filtered * 4095.0 / 3100.0; // convert in ADC Digits
  adc_temp = (uint16_t)adc_values.tempNTC_degree;
  adc_values.tempNTC_degree = ((float)adc_temp * (float)adc_temp * (float)adc_temp * (-2.87638e-9)) + ((float)adc_temp * (float)adc_temp * (2.01243e-5)) + ((-0.0702) * (float)adc_temp) + 109.013;
}

bool bw_check_end_reached_current(void) {
  if (adc_values.current_mA_filtered >= BW_STOP_CURRENT) {
    stop_detection.current_counter++;
    if (stop_detection.current_counter > BW_STOP_CURRENT_COUNTS) {
      DEBUG_INFO("Finished: current:" + String(adc_values.current_mA_filtered) + " above " + String((float)BW_STOP_CURRENT));
      bw_log_event();
      return true;
    }
  } else {
    if (stop_detection.current_counter > 0) {
      stop_detection.current_counter--;
    }
  }
  return false;
}

bool bw_check_end_reached_speed(void) {
  if (abs(bw_motorState.speed) < BW_STOP_SPEED) {
    stop_detection.speed_counter++;
    if (stop_detection.speed_counter >= BW_STOP_SPEED_COUNTS) {
      DEBUG_INFO("Finished: Speed:" + String(abs(bw_motorState.speed)) + " below " + String((float)BW_STOP_SPEED));
      bw_log_event();
      return true;
    }
  } else {
    if (stop_detection.speed_counter > 1) {
      stop_detection.speed_counter--;
    }
  }
  return false;
}

bool bw_check_end_reached(void) {
  return bw_check_end_reached_current() || bw_check_end_reached_speed();
}

bool motorSlowedDown(void) {
  if ((bw_motorState.speed < BW_STOP_SPEED) ||
      (bw_motorState.power == bw_motorState.targetPower)) {
    stop_detection.speed_counter++;
    if (stop_detection.speed_counter >= BW_STOP_SPEED_COUNTS) {
      if(bw_motorState.speed < BW_STOP_SPEED){
        DEBUG_INFO("Decel End Finished: Speed:" + String(bw_motorState.speed) + " below " + String((float)BW_STOP_SPEED));
      } else {
        DEBUG_INFO("Decel End Finished: M_Power:" + String(bw_motorState.power) + " reached target " + String((float)bw_motorState.targetPower));
      }
      bw_log_event();
      return true;
    }
  } else {
    if (stop_detection.speed_counter > 1) {
      stop_detection.speed_counter--;
    }
  }
}

bool bw_safety_protection(void) {
  // Under voltage protection
  if (adc_values.BatteryVoltage_filtered <= BW_STOP_V_BAT) {
    DEBUG_ERROR("Under Voltage: V BAT:" + String(adc_values.BatteryVoltage_filtered) + " below " + String((float)BW_STOP_V_BAT) + "V");
    bw_log_event();
    return true;
  }

  //Temperature protection
  if (adc_values.tempNTC_degree > BW_STOP_T_MAX) {
    DEBUG_ERROR("Over Temperature: T NTC:" + String(adc_values.tempNTC_degree) + "above " + String((float)BW_STOP_T_MAX) + "DEG");
    bw_log_event();
    return true;
  }

  return false;
}

static inline bool bw_readGroundSwitchRaw(void)
{
  bool level = digitalRead(SW_GROUND_PIN);
#ifdef BW_GROUND_SWITCH_INVERTED
  level = !level;
#endif
  return level;
}

static inline bool bw_readCableLooseRaw(void)
{
  bool level = digitalRead(SW_CABLE_LOOSE_PIN);
#ifdef BW_CABLE_LOOSE_INVERTED
  level = !level;
#endif
  return level;
}

void bw_filterInit(BwFilter* f, uint16_t threshold)
{
  f->count     = 0;
  f->threshold = threshold;
  f->state     = false;
}

void bw_filterUpdate(BwFilter* f, bool raw)
{
  uint16_t max = f->threshold * 2;
  if (raw) {
    if (f->count < max) {
      f->count++;
    }
  } else {
    if (f->count > 0) {
      f->count--;
    }
  }
  f->state = (f->count >= f->threshold);
}

void bw_buttonUpdate(ButtonRuntime* btn, bool level)
{
  btn->event = BTN_EVT_NONE;
  btn->rawLevel = level;

  switch (btn->state) {

    case BTN_IDLE:
      if (level) {
        btn->debounceCnt = 0;
        btn->state = BTN_DEBOUNCE;
      }
      break;

    case BTN_DEBOUNCE:
      if (level) {
        if (++btn->debounceCnt >= BW_BTN_DEBOUNCE_TICKS) {
          btn->holdCnt = 0;
          btn->state = BTN_PRESSED;
        }
      } else {
        btn->state = BTN_IDLE;
      }
      break;

    case BTN_PRESSED:
      if (!level) {
        btn->event = BTN_EVT_SHORT;
        btn->state = BTN_IDLE;
      } else if (++btn->holdCnt >= BW_BTN_HOLD_TICKS) {
        btn->event = BTN_EVT_LONG;
        btn->state = BTN_HELD;
      }
      break;

    case BTN_HELD:
      if (!level) {
          btn->event = BTN_EVT_RELEASE;
        btn->state = BTN_IDLE;
      }
      break;
  }
}

bool eventStopRequested(void)
{
  // Stop cleaning by pressing winding-in button
  if (lastUserCommand == CMD_CLEANING &&
      bw_btnIn.state == BTN_PRESSED) {
    DEBUG_WARNING("Stop requested: cleaning");
    return true;
  }

  // Stop winding-in by pressing cleaning button
  if (lastUserCommand == CMD_WINDING_IN &&
      bw_btnOut.state == BTN_PRESSED) {
    DEBUG_WARNING("Stop requested: winding in");
    return true;
  }
  return false;
}


bool groundModeActive(void) {
#ifdef BW_ENABLE_GROUND_MODE
  return bw_groundSwitchFilter.state;
#else
  return false;
#endif
}


void changeMode(BW_MODE newMode)
{
  DEBUG_INFO("FSM transition: " + String(bw_ModeToString(bw_currentMode)) + " -> " + String(bw_ModeToString(newMode)));
  bw_currentMode = newMode;
  const BW_ModeConfig& cfg = bw_modeConfig[newMode];

  bw_modeStartTime = millis();

  // Motor entry action
  bw_motorInit(cfg.motorCmd);

  // LED entry action
  bw_ledInit(cfg.ledCmd);

  bw_log_event();
}


// Handle global transitions with highest priority
bool handleGlobalTransitions(const BW_ModeConfig& cfg) {
  // User stop request (opposite button)
  if (cfg.enableStopRequest && eventStopRequested()) {
    changeMode(M_STOP);
    return true;
  }

  // System error conditions
  if (cfg.enableSafetyProtection && bw_safety_protection()) {
    changeMode(M_ERROR);
    return true;
  }

  bool endCheckAllowed = !cfg.requireMinTimeForEndCheck || (millis() - bw_modeStartTime) >= cfg.minTime;
  bool endReached = false;

  if (endCheckAllowed) {
    if (cfg.enableEndCheckCurrent) {
      endReached = bw_check_end_reached_current();
    }
    if (!endReached && cfg.enableEndCheckSpeed) {
      endReached = bw_check_end_reached_speed();
    }
  }

  if ((cfg.enableEndCheckCurrent || cfg.enableEndCheckSpeed) && endReached) {
    changeMode(M_FINISHED);
    return true;
  }

  // Max time check
  if (cfg.enableMaxTimeCheck && cfg.maxTime > 0 && (millis() - bw_modeStartTime) > cfg.maxTime) {
    changeMode(M_ERROR);
    return true;
  }

  // Wing Tip check
  if (cfg.enableWingTipCheck && bw_motorState.position_mm >= (positionConfig.wingTip + 10)) {
    DEBUG_ERROR("Exceeded wing tip: Position:" + String(bw_motorState.position_mm) + "mm above " + String(positionConfig.wingTip) + "mm");
    bw_log_event();
    changeMode(M_ERROR);
    return true;
  }

  return false;  // No global transition taken
}

void stateIdle(const BW_ModeConfig& cfg) {
  // Waiting for user input

  if (groundModeActive() && (bw_btnOut.event == BTN_EVT_LONG)) {
    lastUserCommand = CMD_CLEANING;   // ground out is still an "out" operation
    changeMode(M_GROUND_OUT);
  }
  else if ((bw_btnIn.event == BTN_EVT_LONG) || (bw_btnIn.state == BTN_HELD)) {
    lastUserCommand = CMD_WINDING_IN;
    changeMode(M_EMERGENCY_IN);
  }
  else if (bw_btnIn.event == BTN_EVT_SHORT) {
    lastUserCommand = CMD_WINDING_IN;
    changeMode(M_WINDING_IN);
  }
  else if ((!groundModeActive()) && ((bw_btnOut.event == BTN_EVT_SHORT) || (bw_btnOut.state == BTN_HELD))) {
    lastUserCommand = CMD_CLEANING;
    changeMode(M_START_CLEAN_OUT);
  }
}

void stateReferenceIn(const BW_ModeConfig& cfg) {
  bool endReached = bw_check_end_reached();
  bool minTimeElapsed = (millis() - bw_modeStartTime) >= cfg.minTime;

  // Transition condition: BOTH must be true
  if (endReached && minTimeElapsed) {
    bw_encoderSetZero();
    changeMode(cfg.defaultNext);
  }
}

void stateStartCleanOut(const BW_ModeConfig& cfg) {

  if (bw_cableLooseFilter.state) {
    changeMode(M_DECEL_LOOSE);
    return;
  }

  if (bw_motorState.position_mm >= positionConfig.slowZoneStartOut) {
    changeMode(cfg.defaultNext);
  }
}

void stateCleaning(const BW_ModeConfig& cfg) {
  if (bw_cableLooseFilter.state) {
    changeMode(M_DECEL_LOOSE);
    return;
  }

  if (bw_motorState.position_mm >= positionConfig.slowZoneWingTip) {
    changeMode(cfg.defaultNext);
  }
}

void stateDecelLoose(const BW_ModeConfig& cfg) {
  if (bw_cableLooseFilter.state == false) {
    changeMode(M_START_CLEAN_OUT);
  } else if (wiggleTimingConfig.decelLooseToWiggleTime_ms > 0 && motorSlowedDown() && (millis() - bw_modeStartTime) >= wiggleTimingConfig.decelLooseToWiggleTime_ms) {
    changeMode(M_WIGGLE_IN);
  }
}

void stateDecelLooseGround(const BW_ModeConfig& cfg) {
  if (bw_cableLooseFilter.state) {
    // In ground mode, the motor remains stopped while the cable is loose.
    return;
  }

  // Cable is OK again, restart ground out using the ground mode motor config.
  changeMode(M_GROUND_OUT);
}

void stateWiggleIn(const BW_ModeConfig& cfg) {
  static uint32_t wigglePhaseStartTime = 0;
  static uint32_t totalWiggleStartTime = 0;
  static bool initialized = false;

  // Initialize wiggle IN state
  if (!initialized) {
    wigglePhaseStartTime = millis();
    if (totalWiggleStartTime == 0) {
      totalWiggleStartTime = millis();  // Start total timeout on first entry
    }
    MotorCommand cmd = {IN, cfg.motorCmd.startPower, cfg.motorCmd.targetPower, cfg.motorCmd.rampTime};
    bw_motorInit(cmd);
    initialized = true;
  }

  uint32_t elapsedInPhase = millis() - wigglePhaseStartTime;
  uint32_t totalElapsed = millis() - totalWiggleStartTime;

  // Check if total wiggle timeout has been reached
  if (totalElapsed >= wiggleTimingConfig.totalWiggleTimeout_ms) {
    // Total timeout reached, exit wiggle mode
    initialized = false;
    totalWiggleStartTime = 0;  // Reset for next time
    changeMode(M_CLEANING);
    return;
  }

  // Retract (IN) phase for specified duration
  if (elapsedInPhase >= wiggleTimingConfig.inDuration_ms) {
    // Switch to OUT phase
    initialized = false;
    changeMode(cfg.defaultNext);  // Should be M_WIGGLE_OUT
  }
}

void stateWiggleOut(const BW_ModeConfig& cfg) {
  static uint32_t wigglePhaseStartTime = 0;
  static uint32_t totalWiggleStartTime = 0;
  static int32_t straightStartPosition = 0;
  static bool initialized = false;

  // Initialize wiggle OUT state
  if (!initialized) {
    wigglePhaseStartTime = millis();
    if (totalWiggleStartTime == 0) {
      totalWiggleStartTime = millis();  // Start total timeout on first entry
    }
    MotorCommand cmd = {OUT, cfg.motorCmd.startPower, cfg.motorCmd.targetPower, cfg.motorCmd.rampTime};
    bw_motorInit(cmd);
    straightStartPosition = bw_motorState.position_mm;
    initialized = true;
  }

  uint32_t elapsedInPhase = millis() - wigglePhaseStartTime;
  uint32_t totalElapsed = millis() - totalWiggleStartTime;

  // Check if total wiggle timeout has been reached
  bool totalTimeoutReached = (totalElapsed >= wiggleTimingConfig.totalWiggleTimeout_ms);

  // Extend (OUT) phase

  // Check if we've reached minimum retract time to check cable status
  if (elapsedInPhase >= wiggleTimingConfig.minRetractTime_ms) {
    if (bw_cableLooseFilter.state && !totalTimeoutReached) {
      // Cable is loose and total timeout not reached, switch back to IN
      initialized = false;
      changeMode(M_WIGGLE_IN);
      return;
    } else if (!bw_cableLooseFilter.state) {
      // Cable is tight, check if it stays tight during extension
      if (bw_motorState.position_mm - straightStartPosition > positionConfig.wiggleStraightDistance) {
        // Cable stayed tight for the required distance, exit wiggle mode
        initialized = false;
        totalWiggleStartTime = 0;  // Reset for next time
        changeMode(cfg.defaultNext);  // Should be M_CLEANING
        return;
      }
    }
  }

  // Check if OUT phase duration has been reached
  if (elapsedInPhase >= wiggleTimingConfig.outDuration_ms) {
    if (totalTimeoutReached) {
      // Total timeout reached, exit wiggle mode
      initialized = false;
      totalWiggleStartTime = 0;  // Reset for next time
      changeMode(M_CLEANING);
    } else {
      // Switch back to IN
      initialized = false;
      changeMode(M_WIGGLE_IN);
    }
  }
}

void stateDecelEnd(const BW_ModeConfig& cfg) {
  if (motorSlowedDown() ||
      (bw_motorState.position_mm >= positionConfig.wingTip)) {
    changeMode(cfg.defaultNext);
  }
}

void stateWindingIn(const BW_ModeConfig& cfg) {
  if ((bw_motorState.position_mm <= positionConfig.windingInDecelDistance) &&
      (abs(bw_motorState.speed) >= positionConfig.windingInDecelSpeed)) {
    changeMode(M_WINDING_IN_DECEL);
    return;
  }
}

void stateWindingInDecel(const BW_ModeConfig& cfg) {
  // Deceleration handled by global timeout and end-check logic
}

void stateGroundOut(const BW_ModeConfig& cfg) {
  // Ground / maintenance outward movement only

  if (bw_cableLooseFilter.state) {
    changeMode(M_DECEL_LOOSE_GROUND);
    return;
  }

  if (bw_motorState.position_mm >= positionConfig.groundOutMax) {
    changeMode(cfg.defaultNext);
  }
}

void stateFinished(const BW_ModeConfig& cfg) {
  lastUserCommand = CMD_NONE;
  if ((millis() - bw_modeStartTime) > cfg.maxTime) {
    bw_encoderSetZero();
    changeMode(cfg.defaultNext);
  }
}

void stateEmergencyIn(const BW_ModeConfig& cfg) {
  if (bw_btnIn.state == BTN_IDLE) {    
    if (bw_check_end_reached()) {
      changeMode(cfg.defaultNext);
      return;
    }
  }
}

void stateStop(const BW_ModeConfig& cfg) {
  lastUserCommand = CMD_NONE;
  if ((millis() - bw_modeStartTime) > cfg.maxTime) {
    changeMode(cfg.defaultNext);
  }
}

void stateError(const BW_ModeConfig& cfg) {
  lastUserCommand = CMD_NONE;
  if ((millis() - bw_modeStartTime) > cfg.maxTime) {
    changeMode(cfg.defaultNext);
  }
}

void bw_processFSM()
{
  // ------------------------------------------------------------

  // Fetch configuration for current mode
  const BW_ModeConfig& cfg = bw_modeConfig[bw_currentMode];

  // 1. Global priority transitions
  // Emergency, Stop and Error have highest priority
  // ------------------------------------------------------------
  if (handleGlobalTransitions(cfg)) {
    return;
  }

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
    case M_WIGGLE_IN:             stateWiggleIn(cfg);           break;
    case M_WIGGLE_OUT:            stateWiggleOut(cfg);          break;
    case M_DECEL_END:             stateDecelEnd(cfg);           break;
    case M_WINDING_IN:            stateWindingIn(cfg);          break;
    case M_WINDING_IN_DECEL:      stateWindingInDecel(cfg);     break;
    case M_GROUND_OUT:            stateGroundOut(cfg);          break;
    case M_DECEL_LOOSE_GROUND:    stateDecelLooseGround(cfg);   break;
    case M_FINISHED:              stateFinished(cfg);           break;
    case M_EMERGENCY_IN:          stateEmergencyIn(cfg);        break;
    case M_STOP:                  stateStop(cfg);               break;
    case M_ERROR:                 stateError(cfg);              break;
    default:                      changeMode(M_ERROR);          break;
  }
}

void bw_Task1_fast(void* parameter) {
  const TickType_t taskPeriod = pdMS_TO_TICKS(BW_TASK_FAST_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  bw_encoder_init();
  for (;;) {

#if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
    uint32_t t_start = micros();
#endif (DEBUG_LEVEL >= DEBUG_LEVEL_INFO)

    bw_filterUpdate(&bw_cableLooseFilter, bw_readCableLooseRaw());
    bw_filterUpdate(&bw_btnInFilter,  !digitalRead(BTN_IN_PIN));    // Inverted
    bw_filterUpdate(&bw_btnOutFilter, !digitalRead(BTN_OUT_PIN));   // Inverted
    bw_adcUpdateRotatingMilliVolts();// vbat / ntc
    bw_read_motor_current();
    bw_motorUpdate();  // motor ramp + HW output    
    
#if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
    uint32_t t_exec = micros() - t_start;
    static uint32_t cnt = 0;
    if (++cnt % 2000 == 0) {
      DEBUG_INFO("t Task1: " + String(t_exec));
    }
#endif

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void bw_Task2_slow(void* parameter) {
  const TickType_t taskPeriod = pdMS_TO_TICKS(BW_TASK_SLOW_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  //bw_test_Motor();
  for (;;) {

#if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
    uint32_t t_start = micros();
#endif

#ifdef BW_ENABLE_GROUND_MODE
    bw_filterUpdate(&bw_groundSwitchFilter, bw_readGroundSwitchRaw());
#endif

    bw_convert_ADCs_slow();
    bw_encoderRead();
    bw_buttonUpdate(&bw_btnIn, bw_btnInFilter.state);
    bw_buttonUpdate(&bw_btnOut, bw_btnOutFilter.state);
    bw_btn_log();
    bw_processFSM();
    bw_ledUpdate();   // LED blink

#if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
    uint32_t t_exec = micros() - t_start;
    static uint32_t cnt = 0;
    if (++cnt % 200 == 0) {
      DEBUG_INFO("t Task2: " + String(t_exec));
    }
#endif

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void bw_rgbLed_init(void) {
  pinMode(RGB_LED_PIN, OUTPUT);
  digitalWrite(RGB_LED_PIN, 0);
}

void bw_init(void) {
  DEBUG_INFO("Init BugWiper:");
  pinMode(SW_CABLE_LOOSE_PIN, INPUT_PULLUP);
  pinMode(SW_GROUND_PIN, INPUT_PULLUP);
  pinMode(BTN_IN_PIN, INPUT_PULLUP);
  pinMode(BTN_OUT_PIN, INPUT_PULLUP);
  //analogSetPinAttenuation(MOTOR_CURRENT_SENSE_PIN, ADC_6db);
  bw_ADC_filter_init();

  // Initialize button states
  bw_btnIn.state = BTN_IDLE;
  bw_btnIn.event = BTN_EVT_NONE;
  bw_btnIn.debounceCnt = 0;
  bw_btnIn.holdCnt = 0;
  bw_btnIn.rawLevel = false;

  bw_btnOut.state = BTN_IDLE;
  bw_btnOut.event = BTN_EVT_NONE;
  bw_btnOut.debounceCnt = 0;
  bw_btnOut.holdCnt = 0;
  bw_btnOut.rawLevel = false;

  // Initialize all filters BEFORE tasks start (avoid race condition)
  bw_filterInit(&bw_cableLooseFilter, BW_SENSOR_FILTER_THRESHOLD);
  bw_filterInit(&bw_btnInFilter,  BW_BTN_FILTER_THRESHOLD);
  bw_filterInit(&bw_btnOutFilter, BW_BTN_FILTER_THRESHOLD);
#ifdef BW_ENABLE_GROUND_MODE
  bw_filterInit(&bw_groundSwitchFilter, BW_SENSOR_FILTER_THRESHOLD);
  if (bw_readGroundSwitchRaw()){
    DEBUG_INFO("Ground Mode Active");
    bw_rgbLedWrite(ORANGE);
  } else {
    DEBUG_INFO("Ground Mode NOT Active");
    bw_rgbLedWrite(GREEN);
  }
  delay(500);
  bw_rgbLedWrite(BLACK);
  delay(200);
#endif

  //bw_encoder_init();
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
  
  // Explicitly set initial FSM state
  changeMode(M_IDLE);
  
  xTaskCreate(bw_Task1_fast, "BW_T1_fast", 1024 * 4, NULL, 3, NULL);
  xTaskCreate(bw_Task2_slow, "BW_T2_alow", 1024 * 8, NULL, 3, NULL);
}




