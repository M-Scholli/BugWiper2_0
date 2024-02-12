#include <dummy.h>

#include <stdint.h>
#include <EEPROM.h>

#define DUAL_MOTOR_CONTROLLER 1

//Taster Pins
#define Putzen_A_PIN 5
#define Ein_Ziehen_A_PIN 18
#define Putzen_B_PIN 2
#define Ein_Ziehen_B_PIN 3
#define F_Fest_A_PIN 11
#define F_Lose_A_PIN 12
#define F_Fest_B_PIN 34
#define F_Lose_B_PIN 35
#define SAVE_PIN 4  // Sicherheitsschalter deaktiviert BugWiper
//Status LED
#define LED_A_PIN 13
#define LED_B_PIN 36
#define LED_T_P 500  //Zeit zum blinken
#define LED_T_E 250
//Motor Pins
#define Motor_A_IN1 25
#define Motor_A_IN2 26
#define Motor_A_EN 27  //PWM Pin
#define Motor_B_IN1 1
#define Motor_B_IN2 14
#define Motor_B_EN 10  //PWM Pin
// Kalibrierung des Putzvorganges
#define T_Taster_Lang 400    // Zeit in ms für langen Tastendruck
#define EINZIEH_MAX_P 255    //Max Power Putzen
#define EINZIEH_MAX_E 255    //Max Motorpower beim Einziehen
#define EINZIEH_MAX_GND 255  //Max Motorpower am Boden
#define BREMSE_START 210     //Motorpower bremsen startwert
#define BREMSE_MAX 255       //Max Motorpower Bremsen
#define BREMSE_L 240
#define VERZOGER_B 2      //Wie viel bis zum erh�hen beim Bremsen in 250�s
#define VERZOGER_P 80     //Verz�gern Putzen
#define VERZOGER_E 20     //Verzögern Einziehen
#define VERZOGER_L 30     //Verzögern Lose
#define T_MIN_P 300       //Minimale Putzzeit[ms]
#define T_MAX_P 90000     //Maximale Putzzeit
#define T_MAX_E 50000     //Maximale festziehzeit //erh�hen der einziehzeit
#define START_POWER_P 30  //Motorpower bei losfahren Putzen
#define START_POWER_F 70
#define START_POWER_L 60    // Start Power nach losem Seil
#define TASTER_Debounce 50  //ms zum Taster enstoeren
//EEPROM Speicherbereich
#define eeRichtung_A 0
#define eeRichtung_B 1
//PWM Configuration
#define PWM_FREQ 10000
#define PWM_RESOLUTION_BITS 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define PWM_PIN_A 0
#define PWM_PIN_B 1

//Globale Variablen
int8_t direction_of_rotation_A;
int8_t direction_of_rotation_A_old;
int8_t cable_loose_a = 0;
uint8_t motor_power_a = 0;
uint16_t timer_button_long_press_a = 0;  // Time since start of long pressing button A
uint8_t timer_motor_power_a = 0;         //Motor PWM
uint16_t timer_LED_a = 0;                //LED
uint32_t timer_start_cleaning_a = 0;     // T_MIN , T_MAX
unsigned long t_timer = 0;
uint8_t pwmMax_A = 0;
uint8_t pwmVerzoger_A = 0;
uint8_t timer_button_cable_loose_a = 0;
uint8_t timer_cable_tight_a = 0;
uint8_t timer_button_winding_in_a = 0;
uint8_t timer_button_start_cleaning_a = 0;
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
uint8_t pwmVerzoger_B = 0;
uint8_t timer_button_cable_loose_b = 0;
uint8_t timer_cable_tight_b = 0;
uint8_t timer_button_winding_in_b = 0;
uint8_t timer_button_start_cleaning_b = 0;
#endif
/*Statusanzeige vom Putzvorgang
 0 = Warten auf Tastendruck
 1 = ganzer Putzvorgang
 2 = Seil einziehen
 3 = warten auf Taster reset
 4 = putzen fertig
 5 = einziehen fertig
 6 = ERROR
 7 = Stopp
 */
uint8_t state_cleaning_a = 0;
uint8_t state_cleaning_b = 0;

void PWM_inti(void) {
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  ledcAttachPin(Motor_A_EN, PWM_CHANNEL_A);
  ledcAttachPin(Motor_B_EN, PWM_CHANNEL_B);
}


void eeprom_update_byte(int adresse, uint8_t wert) {
  if (EEPROM.read(adresse) != wert) {
    EEPROM.write(adresse, wert);
  }
}

// Schaltet den Motor 1= richtung 1;	2=richtung 2;	3=stopp;
void motor_a(uint8_t a) {
  switch (a) {
    case 1:
      {
        digitalWrite(Motor_A_IN2, 0);
        digitalWrite(Motor_A_IN1, 1);
      }
      break;

    case 2:
      {
        digitalWrite(Motor_A_IN1, 0);
        digitalWrite(Motor_A_IN2, 1);
      }
      break;

    case 3:
      {
        digitalWrite(Motor_A_IN2, 0);
        digitalWrite(Motor_A_IN1, 0);
      }
      break;
  }
}

void motor_b(uint8_t a) {
  switch (a) {
    case 1:
      {
        digitalWrite(Motor_B_IN2, 0);
        digitalWrite(Motor_B_IN1, 1);
      }
      break;

    case 2:
      {
        digitalWrite(Motor_B_IN1, 0);
        digitalWrite(Motor_B_IN2, 1);
      }
      break;

    case 3:
      {
        digitalWrite(Motor_B_IN2, 0);
        digitalWrite(Motor_B_IN1, 0);
      }
      break;
  }
}

void set_motorpower_a(void) {
  if (timer_motor_power_a >= pwmVerzoger_A) {
    if (motor_power_a < pwmMax_A) {
      motor_power_a++;
    }
    timer_motor_power_a = 0;
  }
  ledcWrite(PWM_CHANNEL_A, motor_power_a);
}

void set_motorpower_b(void) {
  if (timer_motor_power_b >= pwmVerzoger_B) {
    if (motor_power_b < pwmMax_B) {
      motor_power_b++;
    }
    timer_motor_power_b = 0;
  }
  ledcWrite(PWM_CHANNEL_B, motor_power_b);
}

void set_bremse_a(void) {
  timer_motor_power_a = 0;
  motor_power_a = BREMSE_START;
  pwmVerzoger_A = VERZOGER_B;
  pwmMax_A = BREMSE_MAX;
  motor_a(3);
  state_cleaning_a = 7;
}

void set_bremse_b(void) {
  timer_motor_power_b = 0;
  motor_power_b = BREMSE_START;
  pwmVerzoger_B = VERZOGER_B;
  pwmMax_B = BREMSE_MAX;
  motor_b(3);
  state_cleaning_b = 7;
}

void set_einziehen_A(void) {
  state_cleaning_a = 2;
  timer_LED_a = 0;
  timer_motor_power_a = 0;
  timer_start_cleaning_a = 0;
  motor_power_a = START_POWER_F;
  pwmVerzoger_A = VERZOGER_E;
  pwmMax_A = EINZIEH_MAX_E;
  if (direction_of_rotation_A == 1) {
    motor_a(2);
    direction_of_rotation_A_old = 2;
  } else {
    motor_a(1);
    direction_of_rotation_A_old = 1;
  }
}

void set_einziehen_B(void) {
  state_cleaning_b = 2;
  timer_LED_b = 0;
  timer_motor_power_b = 0;
  timer_start_cleaning_b = 0;
  motor_power_b = START_POWER_F;
  pwmVerzoger_B = VERZOGER_E;
  pwmMax_B = EINZIEH_MAX_E;
  if (direction_of_rotation_B == 1) {
    motor_b(2);
    direction_of_rotation_B_old = 2;
  } else {
    motor_b(1);
    direction_of_rotation_B_old = 1;
  }
}

void set_putzen_A(void) {
  state_cleaning_a = 1;
  timer_motor_power_a = 0;
  timer_LED_a = 0;
  timer_start_cleaning_a = 0;
  motor_power_a = START_POWER_P;
  pwmVerzoger_A = VERZOGER_P;
  pwmMax_A = EINZIEH_MAX_P;
  motor_a(direction_of_rotation_A);
  timer_button_long_press_a = 0;
  direction_of_rotation_A_old = direction_of_rotation_A;
}

void set_putzen_B(void) {
  state_cleaning_b = 1;
  timer_motor_power_b = 0;
  timer_LED_b = 0;
  timer_start_cleaning_b = 0;
  motor_power_b = START_POWER_P;
  pwmVerzoger_B = VERZOGER_P;
  pwmMax_B = EINZIEH_MAX_P;
  motor_b(direction_of_rotation_B);
  timer_button_long_press_b = 0;
  direction_of_rotation_B_old = direction_of_rotation_B;
}

void init_io(void) {
  pinMode(F_Fest_A_PIN, INPUT_PULLUP);
  pinMode(F_Lose_A_PIN, INPUT_PULLUP);
  pinMode(F_Fest_B_PIN, INPUT_PULLUP);
  pinMode(F_Lose_B_PIN, INPUT_PULLUP);
  pinMode(SAVE_PIN, INPUT_PULLUP);
  pinMode(Motor_A_EN, OUTPUT);
  pinMode(Motor_A_IN1, OUTPUT);
  pinMode(Motor_A_IN2, OUTPUT);
  pinMode(Motor_B_EN, OUTPUT);
  pinMode(Motor_B_IN1, OUTPUT);
  pinMode(Motor_B_IN2, OUTPUT);
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(Ein_Ziehen_A_PIN, INPUT_PULLUP);
  pinMode(Putzen_A_PIN, INPUT_PULLUP);
  pinMode(Ein_Ziehen_B_PIN, INPUT_PULLUP);
  pinMode(Putzen_B_PIN, INPUT_PULLUP);
  digitalWrite(Motor_A_IN2, 1);
  digitalWrite(Motor_A_IN1, 1);
  digitalWrite(Motor_B_IN2, 1);
  digitalWrite(Motor_B_IN1, 1);
  digitalWrite(LED_A_PIN, 0);
  digitalWrite(LED_B_PIN, 0);
}

void lese_richtung(void) {
  direction_of_rotation_A = EEPROM.read(eeRichtung_A);
  direction_of_rotation_B = EEPROM.read(eeRichtung_B);
  if (direction_of_rotation_A != 1 && direction_of_rotation_A != 2) {
    direction_of_rotation_A = 1;
    eeprom_update_byte(eeRichtung_A, direction_of_rotation_A);
  }
  if (direction_of_rotation_B != 1 && direction_of_rotation_B != 2) {
    direction_of_rotation_B = 1;
    eeprom_update_byte(eeRichtung_B, direction_of_rotation_B);
  }
}

void schreibe_richtung(void) {
  eeprom_update_byte(eeRichtung_A, direction_of_rotation_A);
  eeprom_update_byte(eeRichtung_B, direction_of_rotation_B);
}

void aender_richtung_A(void) {
  if (direction_of_rotation_A == 1) {
    direction_of_rotation_A = 2;
  } else if (direction_of_rotation_A == 2) {
    direction_of_rotation_A = 1;
  }
  schreibe_richtung();
}

void aender_richtung_B(void) {
  if (direction_of_rotation_B == 1) {
    direction_of_rotation_B = 2;
  } else if (direction_of_rotation_B == 2) {
    direction_of_rotation_B = 1;
  }
  schreibe_richtung();
}

void taster_debounce(void) {
  if (digitalRead(F_Lose_A_PIN) == 0 && timer_button_cable_loose_a < 255) {
    timer_button_cable_loose_a++;
  } else if (digitalRead(F_Lose_A_PIN) && timer_button_cable_loose_a > 0) {
    timer_button_cable_loose_a--;
  }
  if (digitalRead(F_Lose_B_PIN) == 0 && timer_button_cable_loose_b < 255) {
    timer_button_cable_loose_b++;
  } else if (digitalRead(F_Lose_B_PIN) && timer_button_cable_loose_b > 0) {
    timer_button_cable_loose_b--;
  }
  if (digitalRead(F_Fest_A_PIN) == 0 && timer_cable_tight_a < 255) {
    timer_cable_tight_a++;
  } else if (digitalRead(F_Fest_A_PIN) && timer_cable_tight_a > 0) {
    timer_cable_tight_a--;
  }
  if (digitalRead(F_Fest_B_PIN) == 0 && timer_cable_tight_b < 255) {
    timer_cable_tight_b++;
  } else if (digitalRead(F_Fest_B_PIN) && timer_cable_tight_b > 0) {
    timer_cable_tight_b--;
  }
  if (digitalRead(Ein_Ziehen_A_PIN) == 0 && timer_button_winding_in_a < 255) {
    timer_button_winding_in_a++;
  } else if (digitalRead(Ein_Ziehen_A_PIN) && timer_button_winding_in_a > 0) {
    timer_button_winding_in_a--;
  }
  if (digitalRead(Ein_Ziehen_B_PIN) == 0 && timer_button_winding_in_b < 255) {
    timer_button_winding_in_b++;
  } else if (digitalRead(Ein_Ziehen_B_PIN) && timer_button_winding_in_b > 0) {
    timer_button_winding_in_b--;
  }
  if (digitalRead(Putzen_A_PIN) == 0 && timer_button_start_cleaning_a < 255) {
    timer_button_start_cleaning_a++;
  } else if (digitalRead(Putzen_A_PIN) && timer_button_start_cleaning_a > 0) {
    timer_button_start_cleaning_a--;
  }
  if (digitalRead(Putzen_B_PIN) == 0 && timer_button_start_cleaning_b < 255) {
    timer_button_start_cleaning_b++;
  } else if (digitalRead(Putzen_B_PIN) && timer_button_start_cleaning_b > 0) {
    timer_button_start_cleaning_b--;
  }
}

//zählt jede ms die Timer hoch
void setTimer(void) {
  if (micros() >= t_timer) {
    timer_start_cleaning_a++;
    timer_motor_power_a++;
    timer_LED_a++;
    timer_start_cleaning_b++;
    timer_motor_power_b++;
    timer_LED_b++;
    if (timer_button_start_cleaning_a >= TASTER_Debounce && state_cleaning_a == 0) {
      timer_button_long_press_a = timer_button_long_press_a + 1;
    }
    if (timer_button_start_cleaning_b >= TASTER_Debounce && state_cleaning_b == 0) {
      timer_button_long_press_b = timer_button_long_press_b + 1;
    }
    taster_debounce();
    t_timer = t_timer + 1000;
  }
}

void tasterAbfrage(void) {
  if (digitalRead(SAVE_PIN) == 0) {
    // Einziehen starten Motor A
    if (timer_button_winding_in_a >= TASTER_Debounce && state_cleaning_a == 0) {
      set_einziehen_A();
    }
    // Putzen starten Motor A
    if (state_cleaning_a == 0 && timer_button_long_press_a >= T_Taster_Lang) {
      set_putzen_A();
    }
    // Einziehen starten Motor B
    if (timer_button_winding_in_b >= TASTER_Debounce && state_cleaning_b == 0) {
      set_einziehen_B();
    }
    // Putzen starten Motor B
    if (state_cleaning_b == 0 && timer_button_long_press_b >= T_Taster_Lang) {
      set_putzen_B();
    }
  }
  // Verhindert, dass nach einem Putzvorgang direkt ein zweiter startet
  if (timer_button_winding_in_a <= 5 && timer_button_start_cleaning_a <= 5
      && state_cleaning_a == 3 && timer_start_cleaning_a >= 200) {
    state_cleaning_a = 0;
  }
  if (timer_button_winding_in_b <= 5 && timer_button_start_cleaning_b <= 5
      && state_cleaning_b == 3 && timer_start_cleaning_b >= 200) {
    state_cleaning_b = 0;
  }
  // reset des Zählers für lange Tastendrücke
  if (timer_button_start_cleaning_a <= 5) {
    timer_button_long_press_a = 0;
  }
  if (timer_button_start_cleaning_b <= 5) {
    timer_button_long_press_b = 0;
  }
}

void check_Ende(void) {
  if (state_cleaning_a == 4) {
    set_bremse_a();
    aender_richtung_A();
    digitalWrite(LED_A_PIN, 0);
  }
  if (state_cleaning_a == 5) {
    set_bremse_a();
    digitalWrite(LED_A_PIN, 0);
  }
  if (state_cleaning_a == 6) {
    set_bremse_a();
    digitalWrite(LED_A_PIN, 1);
  }
  if (state_cleaning_b == 4) {
    set_bremse_b();
    aender_richtung_B();
    digitalWrite(LED_B_PIN, 0);
  }
  if (state_cleaning_b == 5) {
    set_bremse_b();
    digitalWrite(LED_B_PIN, 0);
  }
  if (state_cleaning_b == 6) {
    set_bremse_b();
    digitalWrite(LED_B_PIN, 1);
  }
}

//The setup function is called once at startup of the sketch
void setup() {
  PWM_inti();
  init_io();
  lese_richtung();
}

// The loop function is called in an endless loop
void loop() {
  setTimer();
  tasterAbfrage();
  if (state_cleaning_a == 1 || state_cleaning_a == 2 || state_cleaning_a == 7) {
    set_motorpower_a();
  }
  if (state_cleaning_b == 1 || state_cleaning_b == 2 || state_cleaning_b == 7) {
    set_motorpower_b();
  }
  if (state_cleaning_a == 1) {
    if (timer_button_start_cleaning_a <= TASTER_Debounce) {
      if (timer_start_cleaning_a >= T_MAX_P) {
        state_cleaning_a = 6;
      }
      if ((timer_start_cleaning_a >= T_MIN_P) && timer_cable_tight_a >= TASTER_Debounce) {
        state_cleaning_a = 4;
      }
      if (timer_button_winding_in_a >= TASTER_Debounce) {
        state_cleaning_a = 6;
      }
      if (timer_start_cleaning_a >= T_MIN_P && timer_button_cable_loose_a <= 5 && cable_loose_a == 0) {
        motor_power_a = BREMSE_L;
        cable_loose_a = 1;
        motor_a(3);
        pwmVerzoger_A = VERZOGER_L;
      }
    }
    if (timer_LED_a == LED_T_P) {
      digitalWrite(LED_A_PIN, !digitalRead(LED_A_PIN));
      timer_LED_a = 0;
    }
    if (timer_button_cable_loose_a >= TASTER_Debounce && cable_loose_a == 1) {
      motor_power_a = START_POWER_L;
      motor_a(direction_of_rotation_A_old);
      pwmVerzoger_A = VERZOGER_L;
      cable_loose_a = 0;
    }
  }
  if (state_cleaning_a == 2) {
    if (timer_button_winding_in_a <= TASTER_Debounce) {
      // maximale Einziehzeit erreicht
      if (timer_start_cleaning_a >= T_MAX_E) {
        state_cleaning_a = 6;
      }
      //Stopp bei drücken des Putzen Pins
      if (timer_button_start_cleaning_a >= TASTER_Debounce) {
        state_cleaning_a = 6;
      }
      // Stopp bei erreichen des Fest-Tasters
      if (timer_cable_tight_a >= TASTER_Debounce) {
        state_cleaning_a = 5;
      }
      if (timer_start_cleaning_a >= T_MIN_P && timer_button_cable_loose_a <= 5 && cable_loose_a == 0) {
        motor_power_a = BREMSE_L;
        cable_loose_a = 1;
        motor_a(3);
        pwmVerzoger_A = VERZOGER_L;
      }
    }
    // LED Blinken
    if (timer_LED_a >= LED_T_E) {
      digitalWrite(LED_A_PIN, !digitalRead(LED_A_PIN));
      timer_LED_a = 0;
    }
    if (timer_button_cable_loose_a >= TASTER_Debounce && cable_loose_a == 1) {
      motor_power_a = START_POWER_L;
      motor_a(direction_of_rotation_A_old);
      pwmVerzoger_A = VERZOGER_L;
      cable_loose_a = 0;
    }
  }
  if (state_cleaning_b == 1) {
    if (timer_button_start_cleaning_b <= TASTER_Debounce) {
      if (timer_start_cleaning_b >= T_MAX_P) {
        state_cleaning_b = 6;
      }
      if ((timer_start_cleaning_b >= T_MIN_P) && timer_cable_tight_b >= TASTER_Debounce) {
        state_cleaning_b = 4;
      }
      if (timer_button_winding_in_b >= TASTER_Debounce) {
        state_cleaning_b = 6;
      }
      if (timer_start_cleaning_b >= T_MIN_P && timer_button_cable_loose_b <= 5 && cable_loose_b == 0) {
        motor_power_b = BREMSE_L;
        motor_b(3);
        pwmVerzoger_B = VERZOGER_L;
        cable_loose_b = 1;
      }
    }
    if (timer_LED_b == LED_T_P) {
      digitalWrite(LED_B_PIN, !digitalRead(LED_B_PIN));
      timer_LED_b = 0;
    }
    if (timer_button_cable_loose_b >= TASTER_Debounce && cable_loose_b == 1) {
      motor_power_b = START_POWER_L;
      motor_b(direction_of_rotation_B_old);
      pwmVerzoger_B = VERZOGER_L;
      cable_loose_b = 0;
    }
  }
  if (state_cleaning_b == 2) {
    if (timer_button_winding_in_b <= TASTER_Debounce) {
      // maximale Einziehzeit erreicht
      if (timer_start_cleaning_b >= T_MAX_E) {
        state_cleaning_b = 6;
      }
      //Stopp bei drücken des Putzen Pins
      if (timer_button_start_cleaning_b >= TASTER_Debounce) {
        state_cleaning_b = 6;
      }
      // Stopp bei erreichen des Fest-Tasters
      if (timer_cable_tight_b >= TASTER_Debounce) {
        state_cleaning_b = 5;
      }
      if (timer_start_cleaning_b >= T_MIN_P && timer_button_cable_loose_b <= 5 && cable_loose_b == 0) {
        motor_power_b = BREMSE_L;
        motor_b(3);
        pwmVerzoger_B = VERZOGER_L;
        cable_loose_b = 1;
      }
    }
    // LED Blinken
    if (timer_LED_b >= LED_T_E) {
      digitalWrite(LED_B_PIN, !digitalRead(LED_B_PIN));
      timer_LED_b = 0;
    }
    if (timer_button_cable_loose_b >= TASTER_Debounce && cable_loose_b == 1) {
      motor_power_b = START_POWER_L;
      motor_b(direction_of_rotation_B_old);
      pwmVerzoger_B = VERZOGER_L;
      cable_loose_b = 0;
    }
  }
  // Motor bremsen
  if (state_cleaning_a == 7 && motor_power_a == 255) {
    state_cleaning_a = 3;
    timer_start_cleaning_a = 0;
  }
  if (state_cleaning_b == 7 && motor_power_b == 255) {
    state_cleaning_b = 3;
    timer_start_cleaning_b = 0;
  }
  // Putzen beendet?
  check_Ende();
}
