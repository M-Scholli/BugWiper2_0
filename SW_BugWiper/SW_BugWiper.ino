#include <Arduino.h>
#include <ESP32Encoder.h>
#include "BugWiper.h"

#define USE_WIFI 0
#define ESP32_S3_DevKit 0

#if USE_WIFI
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include "webpages.h"
#include "FS.h"
#include "FFat.h"
#endif

#define FIRMWARE_VERSION "V0.0.2"
#define DEBUG_SERIAL_OUT 2


#if USE_WIFI
#define Wifi_Boot_Pin 4
#endif

// Pin discription
// ESP32-Wroom-32: 
// follwing pins are not allowed to use: 0 (Bootselect); 2 Board LED / must be low on boot; (1 & 3 UART USB-Serial); 
// (5 must be high during boot);(15 Debugging Log on U0TXD During Booting);(6, 7, 8, 9, 10 & 11 connected to Flash);
// Input Only Pins : 34,35,36,39
// ESP32-S3-Wroom-1:
// follwing pins are difficult to use: 0 (Bootselect); 3 (Strapping Pins Floating) ; 19 & 20 (USB-Jtag); 35,36&37 (Octal PSRAM (8MB));
// 39,40,41&42 (JTAG); 43 & 44 (UART 0 for serial debug interface); 45 & 46 (Strapping Pins / Pull-down)  48 Board LED

//Button PINs
#if ESP32_S3_DevKit
  #define BUTTON_CLEANING_A_PIN 5
  #define BUTTON_WINDING_IN_A_PIN 6
  #define SW_CABLE_LOOSE_A_PIN 10
  #define SAFETY_SWITCH_PIN 11  // Saftyswitch to deaktivate the BugWiper
//LED configuration
  #define RGB_BUILD_IN 48
  #define LED_A_PIN 48
//Motor PINs
  #define MOTOR_A_IN1_PIN 12
  #define MOTOR_A_IN2_PIN 13
  #define MOTOR_A_EN_PIN 14  //PWM Pin
  #define MOTOR_A_CURRENT_SENSE_PIN 16
  #define MOTOR_A_ENCODER_1_PIN 1
  #define MOTOR_A_ENCODER_2_PIN 2
#else
  #define BUTTON_CLEANING_A_PIN 21
  #define BUTTON_WINDING_IN_A_PIN 18
  #define SW_CABLE_LOOSE_A_PIN 23
  #define SAFETY_SWITCH_PIN 16  // Saftyswitch to deaktivate the BugWiper
//LED configuration
  #define LED_A_PIN 2
//Motor PINs
  #define MOTOR_A_IN1_PIN 12
  #define MOTOR_A_IN2_PIN 13
  #define MOTOR_A_EN_PIN 14  //PWM Pin
  #define MOTOR_A_CURRENT_SENSE_PIN 36
  #define MOTOR_A_ENCODER_1_PIN 26
  #define MOTOR_A_ENCODER_2_PIN 27
#endif

// Kalibrierung des Putzvorganges
#define TIME_LONG_PRESS 400           //time in ms for long button press
#define MAX_POWER_CLEANING 255        //max power while cleaning
#define MAX_POWER_WINDING_IN 255      //max power while winding in
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

//PWM configuration
#define PWM_CHANNEL_A 0

//Global variables
uint16_t timer_button_long_press_a = 0;  // Time since start of long pressing button A
uint8_t timer_button_cable_loose_a = 0;
uint8_t timer_button_winding_in_a = 0;
uint8_t timer_button_start_cleaning_a = 0;
ESP32Encoder encoder_motor_a;


hw_timer_t *Timer0_Cfg = NULL;
volatile uint16_t counter_timer = 0;

BugWiper Putzi_a(LED_A_PIN, MOTOR_A_CURRENT_SENSE_PIN, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PIN, MOTOR_A_EN_PIN, PWM_CHANNEL_A);


void IRAM_ATTR Timer0_ISR(void) {
  counter_timer = counter_timer+1;
  Putzi_a.read_motor_current();
  if (counter_timer == 10) {
    counter_timer = 0;
    Putzi_a.set_timer();
  }
}

void Encoder_init(void) {
  encoder_motor_a.attachHalfQuad(MOTOR_A_ENCODER_1_PIN, MOTOR_A_ENCODER_2_PIN);
  encoder_motor_a.setCount(0);
}

void Timer_init(void) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Code for version 3.x
  Timer0_Cfg = timerBegin(1000000);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
  timerAlarm(Timer0_Cfg, 100, true, 0);
#else
  // Code for version 2.x
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100, true);
  timerAlarmEnable(Timer0_Cfg);
#endif
  
}

void init_io(void) {
#if (DEBUG_SERIAL_OUT)
  Serial.println("Init PINs A:");
#endif
  pinMode(SW_CABLE_LOOSE_A_PIN, INPUT_PULLUP);
  pinMode(SAFETY_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_WINDING_IN_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CLEANING_A_PIN, INPUT_PULLUP);
#if (DEBUG_SERIAL_OUT)
  Serial.println("Init PINs A complete");
#endif
}

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
}

void read_Buttons(void) {
  if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
    if (timer_button_winding_in_a >= TIME_BUTTON_DEBOUNCE && Putzi_a.state_machine_state == 0) {
      Putzi_a.set_winding_in();
    }
    if (Putzi_a.state_machine_state == 0 && timer_button_long_press_a >= TIME_LONG_PRESS) {
      Putzi_a.set_start_cleaning();
    }
  }
  // prevents a imediate second start cleaning after finish the first one
  if (timer_button_winding_in_a <= 5 && timer_button_start_cleaning_a <= 5
      && Putzi_a.state_machine_state == 3 && Putzi_a.timer_cleaning >= 200) {
    Putzi_a.state_machine_state = 0;
  }
  // reset timer for long press of buttons
  if (timer_button_start_cleaning_a <= 5) {
    timer_button_long_press_a = 0;
  }
}

#if USE_WIFI
// Webserver based on https://github.com/smford/esp32-asyncwebserver-fileupload-example.git
const String default_ssid = "somessid";
const String default_wifipassword = "mypassword";
const String default_httpuser = "admin";
const String default_httppassword = "admin";
const int default_webserverporthttp = 80;

bool shouldReboot = false;  // schedule a reboot
bool shouldUpdate = false;  // schedule a firmware update
bool ConfigMode = false;

AsyncWebServer *server;     // initialise webserver
#endif

//The setup function is called once at startup of the sketch
void setup() {
#if USE_WIFI
  pinMode(Wifi_Boot_Pin, INPUT_PULLUP);
#endif

#if DEBUG_SERIAL_OUT
  Serial.begin(115200);
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("Booting ...");
#endif

#if USE_WIFI
  //   Serial.println("FatFS, formatting");
  // #warning "WARNING ALL DATA WILL BE LOST: FFat.format()"
  //FFat.format();
 init_FAT();

  if (digitalRead(Wifi_Boot_Pin)) {
  
#if DEBUG_SERIAL_OUT
    Serial.println("PIN Config Mode: No Wifi selected");
#endif 

#endif

#if DEBUG_SERIAL_OUT
Serial.println("BugWiper start programm");
#endif

    Encoder_init();
    Putzi_a.init();
    init_io();
    Timer_init();
#if (DEBUG_SERIAL_OUT)
    if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
      Serial.println("SAFETY SWITCH closed");
    } else {
      Serial.println("WARNING!!! SAFETY SWITCH open");
      Serial.println("Close the SAFETY SWITCH to operate the BugWiper");
    }
#endif

#if USE_WIFI
  } else {
   init_wifi();
  }
#endif
}

uint16_t counter_output=0;
// The loop function is called in an endless loop
void loop() {
#if USE_WIFI
  if (ConfigMode) {
   check_wifi_functions(); 
  } else {
#endif    
    read_Buttons();
#if (DEBUG_SERIAL_OUT >= 2)
  if (counter_output > 40000) {
    Serial.println("ADC value = " + String(Putzi_a.ADC_current_sense));
    Serial.println("Encoder count = " + String((int32_t)encoder_motor_a.getCount()));
    counter_output = 0;
  }
  counter_output++;
#endif
#if USE_WIFI
  }
#endif  
}


