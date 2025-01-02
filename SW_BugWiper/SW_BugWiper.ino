#include <Arduino.h>
#include <ESP32Encoder.h>
#include "BugWiper.h"

#define USE_WIFI 1

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

// Pindiscription, ESP32-Wroom: follwing pins are not allowed to use: 0 (Bootselect); 2 Board LED; 1 & 3 (UART 0 for serial debug interface); 5 ?;  6, 7, 8, 9, 10 & 11 (4 MB SPI Flash); 16-17 (PSRAM)
// Pindiscription, ESP32-S3: follwing pins are difficult to use: 0 (Bootselect); 3 (Strapping Pins Floating) ; 19 & 20 (USB); 35,36&37 (Octal PSRAM (8MB)); 39,40,41&42 (JTAG); 43 & 44 (UART 0 for serial debug interface); 45 & 46 (Strapping Pins / Pull-down)  48 Board LED

//Button PINs
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
  counter_timer++;
  Putzi_a.read_motor_current();
  if (counter_timer == 10) {
    counter_timer = 0;
    Putzi_a.calculate(encoder_motor_a.getCount(), 0, 0, 0);
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

// configuration structure
struct Config {
  String ssid;            // wifi ssid
  String wifipassword;    // wifi password
  String httpuser;        // username to access web admin
  String httppassword;    // password to access web admin
  int webserverporthttp;  // http port number for web admin
};

// variables
Config config;              // configuration
bool shouldReboot = false;  // schedule a reboot
bool shouldUpdate = false;  // schedule a firmware update
AsyncWebServer *server;     // initialise webserver

// function defaults
String listFiles(bool ishtml = false);

bool ConfigMode = false;
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
  if (!FFat.begin()) {
    // Note: An error occurs when using the ESP32 for the first time, it needs to be formatted
    //
    //     Serial.println("ERROR: Cannot mount FatFS, Try formatting");
    // #warning "WARNING ALL DATA WILL BE LOST: FFat.format()"
    //     FFat.format();

    if (!FFat.begin()) {
#if DEBUG_SERIAL_OUT
      Serial.println("ERROR: Cannot mount FatFS, Rebooting");
#endif
      rebootESP("ERROR: Cannot mount FatFS, Rebooting");
    }
  }
#if DEBUG_SERIAL_OUT
  Serial.print("FatFS Free: ");
  Serial.println(humanReadableSize(FFat.freeBytes()));
  Serial.print("FatFS Used: ");
  Serial.println(humanReadableSize(FFat.usedBytes()));
  Serial.print("FatFS Total: ");
  Serial.println(humanReadableSize(FFat.totalBytes()));

  Serial.println(listFiles());
#endif

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
    ConfigMode = true;
    digitalWrite(2, HIGH);
    Serial.println("PIN Config Mode:: Start Wifi to enter Config Mode");

    Serial.println("Loading Configuration ...");

    config.ssid = default_ssid;
    config.wifipassword = default_wifipassword;
    config.httpuser = default_httpuser;
    config.httppassword = default_httppassword;
    config.webserverporthttp = default_webserverporthttp;

    Serial.print("\nConnecting to Wifi: ");
    WiFi.softAP(config.ssid.c_str(), config.wifipassword.c_str());
    WiFi.softAPsetHostname(config.ssid.c_str());


    Serial.println("\n\nNetwork Configuration:");
    Serial.println("----------------------");
    Serial.print("         SSID: ");
    Serial.println(WiFi.softAPSSID());
    Serial.print("Wifi Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    Serial.print("          MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("           IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("       Subnet: ");
    Serial.println(WiFi.softAPSubnetMask());
    Serial.println();

    // configure web server
    Serial.println("Configuring Webserver ...");
    server = new AsyncWebServer(config.webserverporthttp);
    configureWebServer();

    // startup web server
    Serial.println("Starting Webserver ...");
    server->begin();
  }
#endif
}

// The loop function is called in an endless loop
void loop() {
#if USE_WIFI
  if (ConfigMode) {
    // reboot if we've told it to reboot
    if (shouldReboot) {
      rebootESP("Web Admin Initiated Reboot");
    }
    if (shouldUpdate) {
      updateESP("Web Admin Initiated Update");
    }
  } else {
#endif    
    read_Buttons();
#if (DEBUG_SERIAL_OUT >= 2)
    Serial.println("ADC value = " + String(Putzi_a.ADC_current_sense));
    Serial.println("Encoder count = " + String((int32_t)encoder_motor_a.getCount()));
#endif
#if USE_WIFI
  }
#endif  
}


