#include <Arduino.h>
#include <ESP32Encoder.h>
#include "BugWiper.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include "webpages.h"
#include "FS.h"
#include "FFat.h"

#define Wifi_Boot_Pin 4
#define FIRMWARE_VERSION "V0.0.2"
#define DUAL_MOTOR_CONTROLLER 0
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
//Motor PINs
#define MOTOR_A_IN1_PIN 12
#define MOTOR_A_IN2_PIN 13
#define MOTOR_A_EN_PIN 14  //PWM Pin
#define MOTOR_A_CURRENT_SENSE_PIN 36
#define MOTOR_A_ENCODER_1_PIN 26
#define MOTOR_A_ENCODER_2_PIN 27
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

//PWM configuration
#define PWM_CHANNEL_A 0
#if (DUAL_MOTOR_CONTROLLER)
#define PWM_CHANNEL_B 1
#endif

//Global variables
uint16_t timer_button_long_press_a = 0;  // Time since start of long pressing button A
uint8_t timer_button_cable_loose_a = 0;
uint8_t timer_button_winding_in_a = 0;
uint8_t timer_button_start_cleaning_a = 0;
ESP32Encoder encoder_motor_a;

#if (DUAL_MOTOR_CONTROLLER)
uint16_t timer_button_long_press_b = 0;  // Time since start of long pressing button B
uint8_t timer_button_cable_loose_b = 0;
uint8_t timer_button_winding_in_b = 0;
uint8_t timer_button_start_cleaning_b = 0;
ESP32Encoder encoder_motor_b;
#endif

hw_timer_t *Timer0_Cfg = NULL;
volatile uint16_t counter_timer = 0;

BugWiper Putzi_a(LED_A_PIN, MOTOR_A_CURRENT_SENSE_PIN, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PIN, MOTOR_A_EN_PIN);

#if (DUAL_MOTOR_CONTROLLER)
BugWiper Putzi_b(LED_B_PIN, MOTOR_B_CURRENT_SENSE_PIN, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PIN, MOTOR_B_EN_PIN);
#endif



void IRAM_ATTR Timer0_ISR(void) {
  counter_timer++;
  Putzi_a.read_motor_current();
#if (DUAL_MOTOR_CONTROLLER)
  Putzi_b.read_motor_current();
#endif
  if (counter_timer == 10) {
    counter_timer = 0;
    Putzi_a.calculate(encoder_motor_a.getCount(), 0, 0, 0);
#if (DUAL_MOTOR_CONTROLLER)
    Putzi_b.calculate(encoder_motor_b.getCount(), 0, 0, 0);
#endif
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
  Timer0_Cfg = timerBegin(1000000);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
  timerAlarm(Timer0_Cfg, 100, true, 0);
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
#if (DUAL_MOTOR_CONTROLLER)
#if (DEBUG_SERIAL_OUT)
  Serial.println("Init PINs B:");
#endif
  pinMode(SW_CABLE_LOOSE_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_WINDING_IN_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CLEANING_B_PIN, INPUT_PULLUP);
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

void read_Buttons(void) {
  if (digitalRead(SAFETY_SWITCH_PIN) == 0) {
    if (timer_button_winding_in_a >= TIME_BUTTON_DEBOUNCE && Putzi_a.state_machine_state == 0) {
      Putzi_a.set_winding_in();
    }
    if (Putzi_a.state_machine_state == 0 && timer_button_long_press_a >= TIME_LONG_PRESS) {
      Putzi_a.set_start_cleaning();
    }
#if (DUAL_MOTOR_CONTROLLER)
    if (timer_button_winding_in_b >= TIME_BUTTON_DEBOUNCE && Putzi_b.state_machine_state == 0) {
      Putzi_b.set_winding_in();
    }
    if (Putzi_b.state_machine_state == 0 && timer_button_long_press_b >= TIME_LONG_PRESS) {
      Putzi_b.set_start_cleaning();
    }
#endif
  }
  // prevents a imediate second start cleaning after finish the first one
  if (timer_button_winding_in_a <= 5 && timer_button_start_cleaning_a <= 5
      && Putzi_a.state_machine_state == 3 && Putzi_a.timer_cleaning >= 200) {
    Putzi_a.state_machine_state = 0;
  }
#if (DUAL_MOTOR_CONTROLLER)
  if (timer_button_winding_in_b <= 5 && timer_button_start_cleaning_b <= 5
      && Putzi_b.state_machine_state == 3 && Putzi_b.timer_cleaning >= 200) {
    Putzi_b.state_machine_state = 0;
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

//The setup function is called once at startup of the sketch
void setup() {
  pinMode(Wifi_Boot_Pin, INPUT_PULLUP);
#if (DEBUG_SERIAL_OUT)
  Serial.begin(115200);
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_VERSION);

  Serial.println("Booting ...");

  Serial.println("Mounting FatFS ...");

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
      Serial.println("ERROR: Cannot mount FatFS, Rebooting");
      rebootESP("ERROR: Cannot mount FatFS, Rebooting");
    }
  }

  Serial.print("FatFS Free: ");
  Serial.println(humanReadableSize(FFat.freeBytes()));
  Serial.print("FatFS Used: ");
  Serial.println(humanReadableSize(FFat.usedBytes()));
  Serial.print("FatFS Total: ");
  Serial.println(humanReadableSize(FFat.totalBytes()));

  Serial.println(listFiles());

  if (digitalRead(Wifi_Boot_Pin)) {
    Serial.println("PIN Config Mode: No Wifi selected");
    Serial.println("BugWiper start programm");
#endif
    Encoder_init();
    Putzi_a.init();
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
}

// The loop function is called in an endless loop
void loop() {
  if (ConfigMode) {
    // reboot if we've told it to reboot
    if (shouldReboot) {
      rebootESP("Web Admin Initiated Reboot");
    }
    if (shouldUpdate) {
      updateESP("Web Admin Initiated Update");
    }
  } else {
    read_Buttons();
#if (DEBUG_SERIAL_OUT >= 2)
    Serial.println("ADC value = " + String(Putzi_a.ADC_current_sense));
    Serial.println("Encoder count = " + String((int32_t)encoder_motor_a.getCount()));
#if (DUAL_MOTOR_CONTROLLER)
    Serial.println("ADC_B value = " + String(Putzi_b.ADC_current_sense));
    Serial.println("Encoder_B count = " + String((int32_t)encoder_motor_b.getCount()));
#endif
#endif
  }
}

// the following is based on https://github.com/smford/esp32-asyncwebserver-fileupload-example.git

void rebootESP(String message) {
  Serial.print("Rebooting ESP32: ");
  Serial.println(message);
  ESP.restart();
}

void updateESP(String message) {
  Serial.print("Rebooting ESP32: ");
  Serial.println(message);
  File firmware = FFat.open("/firmware.bin");
  if (firmware) {
    Serial.println(F("found!"));
    Serial.println(F("Try to update!"));

    //Update.onProgress(progressHandler);
    Update.begin(firmware.size(), U_FLASH);
    Update.writeStream(firmware);
    if (Update.end()) {
      Serial.println(F("Update finished!"));
    } else {
      Serial.println(F("Update error!"));
      Serial.println(Update.getError());
    }

    firmware.close();

    if (FFat.rename("/firmware.bin", "/firmware.bak")) {
      Serial.println(F("Firmware rename succesfully!"));
    } else {
      Serial.println(F("Firmware rename error!"));
    }
    delay(2000);

    ESP.restart();
  }
}

// list all of the files, if ishtml=true, return html rather than simple text
String listFiles(bool ishtml) {
  String returnText = "";
  Serial.println("Listing files stored on FatFS");
  File root = FFat.open("/");
  File foundfile = root.openNextFile();
  if (ishtml) {
    returnText += "<table><tr><th align='left'>Name</th><th align='left'>Size</th><th></th><th></th></tr>";
  }
  while (foundfile) {
    if (ishtml) {
      returnText += "<tr align='left'><td>" + String(foundfile.name()) + "</td><td>" + humanReadableSize(foundfile.size()) + "</td>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'download\')\">Download</button>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'delete\')\">Delete</button></tr>";
    } else {
      returnText += "File: " + String(foundfile.name()) + " Size: " + humanReadableSize(foundfile.size()) + "\n";
    }
    foundfile = root.openNextFile();
  }
  if (ishtml) {
    returnText += "</table>";
  }
  root.close();
  foundfile.close();
  return returnText;
}

// Make size of files human readable
// source: https://github.com/CelliesProjects/minimalUploadAuthESP32
String humanReadableSize(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " kB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}
