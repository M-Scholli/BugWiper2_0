#include "BugWiper2_0.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <EEPROM.h>

//Taster Pins
#define Putzen_A_PIN		5
#define Ein_Ziehen_A_PIN	6
#define Putzen_B_PIN		2
#define Ein_Ziehen_B_PIN	3
#define F_Fest_A_PIN		11
#define F_Lose_A_PIN		12
#define F_Fest_B_PIN		A3
#define F_Lose_B_PIN		A4
#define SAVE_PIN		4	// Sicherheitsschalter deaktiviert BugWiper
//Status LED
#define LED_A_PIN		13
#define LED_B_PIN		A0
#define LED_T_P			500	//Zeit zum blinken
#define LED_T_E			250
//Motor Pins
#define Motor_A_IN1		7
#define Motor_A_IN2		8
#define Motor_A_EN		9 	//PWM Pin
#define Motor_B_IN1		A1
#define Motor_B_IN2		A2
#define Motor_B_EN		10 	//PWM Pin
// Kalibrierung des Putzvorganges
#define T_Taster_Lang		1000    // Zeit in ms für langen Tastendruck
#define EINZIEH_MAX_P		255	//Max Power Putzen
#define EINZIEH_MAX_E		255	//Max Motorpower beim Einziehen
#define EINZIEH_MAX_GND		255 	//Max Motorpower am Boden
#define BREMSE_START		210 	//Motorpower bremsen startwert
#define BREMSE_MAX		255	//Max Motorpower Bremsen
#define VERZOGER_B		2  	//Wie viel bis zum erh�hen beim Bremsen in 250�s
#define VERZOGER_P		60 	//Verz�gern Putzen
#define VERZOGER_E		20	//Verzögern Einziehen
#define T_MIN_P			300	//Minimale Putzzeit[ms]
#define T_MAX_P			90000	//Maximale Putzzeit
#define T_MAX_E			50000 	//Maximale festziehzeit //erh�hen der einziehzeit
#define START_POWER_P		40 	//Motorpower bei losfahren Putzen
#define START_POWER_F		70
#define TASTER_Debounce		50      //ms zum Taster enstoeren
//EEPROM Speicherbereich
#define eeRichtung_A		0
#define eeRichtung_B		1

//Globale Variablen
int8_t motorrichtung_A;
int8_t motorrichtung_B;
uint8_t motorpower_a = 0;
uint8_t motorpower_b = 0;
uint16_t t_taster_lang_a = 0;	// Zeit seit rücken des Langen Tasters
uint8_t t_m_pwm_a = 0; 		//Motor PWM
uint16_t t_led_a = 0; 		//LED
uint32_t t_p_start_a = 0;		// T_MIN , T_MAX
uint16_t t_taster_lang_b = 0;	// Zeit seit rücken des Langen Tasters
uint8_t t_m_pwm_b = 0; 		//Motor PWM
uint16_t t_led_b = 0; 		//LED
uint32_t t_p_start_b = 0;		// T_MIN , T_MAX
unsigned long t_timer = 0;
uint8_t pwmMax_A = 0;
uint8_t pwmVerzoger_A = 0;
uint8_t pwmMax_B = 0;
uint8_t pwmVerzoger_B = 0;
uint8_t t_taster_lose_a = 0;
uint8_t t_taster_lose_b = 0;
uint8_t t_taster_fest_a = 0;
uint8_t t_taster_fest_b = 0;
uint8_t t_taster_einzieh_a = 0;
uint8_t t_taster_einzieh_b = 0;
uint8_t t_taster_putzen_a = 0;
uint8_t t_taster_putzen_b = 0;

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
uint8_t status_putzen_a = 0;
uint8_t status_putzen_b = 0;

void setPwmFrequency(int pin, int divisor)
    {
    byte mode;
    if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
	{
	switch (divisor)
	    {
	case 1:
	    mode = 0x01;
	    break;
	case 8:
	    mode = 0x02;
	    break;
	case 64:
	    mode = 0x03;
	    break;
	case 256:
	    mode = 0x04;
	    break;
	case 1024:
	    mode = 0x05;
	    break;
	default:
	    return;
	    }
	if (pin == 5 || pin == 6)
	    {
	    TCCR0B = TCCR0B & 0b11111000 | mode;
	    }
	else
	    {
	    TCCR1B = TCCR1B & 0b11111000 | mode;
	    }
	}
    else if (pin == 3 || pin == 11)
	{
	switch (divisor)
	    {
	case 1:
	    mode = 0x01;
	    break;
	case 8:
	    mode = 0x02;
	    break;
	case 32:
	    mode = 0x03;
	    break;
	case 64:
	    mode = 0x04;
	    break;
	case 128:
	    mode = 0x05;
	    break;
	case 256:
	    mode = 0x06;
	    break;
	case 1024:
	    mode = 0x7;
	    break;
	default:
	    return;
	    }
	TCCR2B = TCCR2B & 0b11111000 | mode;
	}
    }

void eeprom_update_byte(int adresse, uint8_t wert)
    {
    if (EEPROM.read(adresse) != wert)
	{
	EEPROM.write(adresse, wert);
	}
    }

// Schaltet den Motor 1= richtung 1;	2=richtung 2;	3=stopp;
void motor_a(uint8_t a)
    {
    switch (a)
	{
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

void motor_b(uint8_t a)
    {
    switch (a)
	{
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

void set_motorpower_a(void)
    {
    if (t_m_pwm_a >= pwmVerzoger_A)
	{
	if (motorpower_a < pwmMax_A)
	    {
	    motorpower_a++;
	    }
	t_m_pwm_a = 0;
	}
    analogWrite(Motor_A_EN, motorpower_a);
    }

void set_motorpower_b(void)
    {
    if (t_m_pwm_b >= pwmVerzoger_B)
	{
	if (motorpower_b < pwmMax_B)
	    {
	    motorpower_b++;
	    }
	t_m_pwm_b = 0;
	}
    analogWrite(Motor_B_EN, motorpower_b);
    }

void set_bremse_a(void)
    {
    t_m_pwm_a = 0;
    motorpower_a = BREMSE_START;
    pwmVerzoger_A = VERZOGER_B;
    pwmMax_A = BREMSE_MAX;
    motor_a(3);
    status_putzen_a = 7;
    }

void set_bremse_b(void)
    {
    t_m_pwm_b = 0;
    motorpower_b = BREMSE_START;
    pwmVerzoger_B = VERZOGER_B;
    pwmMax_B = BREMSE_MAX;
    motor_b(3);
    status_putzen_b = 7;
    }

void set_einziehen_A(void)
    {
    status_putzen_a = 2;
    t_led_a = 0;
    t_m_pwm_a = 0;
    t_p_start_a = 0;
    motorpower_a = START_POWER_F;
    pwmVerzoger_A = VERZOGER_E;
    pwmMax_A = EINZIEH_MAX_E;
    if (motorrichtung_A == 1)
	motor_a(2);
    else
	motor_a(1);
    }

void set_einziehen_B(void)
    {
    status_putzen_b = 2;
    t_led_b = 0;
    t_m_pwm_b = 0;
    t_p_start_b = 0;
    motorpower_b = START_POWER_F;
    pwmVerzoger_B = VERZOGER_E;
    pwmMax_B = EINZIEH_MAX_E;
    if (motorrichtung_B == 1)
	motor_b(2);
    else
	motor_b(1);
    }

void set_putzen_A(void)
    {
    status_putzen_a = 1;
    t_m_pwm_a = 0;
    t_led_a = 0;
    t_p_start_a = 0;
    motorpower_a = START_POWER_P;
    pwmVerzoger_A = VERZOGER_P;
    pwmMax_A = EINZIEH_MAX_P;
    motor_a(motorrichtung_A);
    t_taster_lang_a = 0;
    }

void set_putzen_B(void)
    {
    status_putzen_b = 1;
    t_m_pwm_b = 0;
    t_led_b = 0;
    t_p_start_b = 0;
    motorpower_b = START_POWER_P;
    pwmVerzoger_B = VERZOGER_P;
    pwmMax_B = EINZIEH_MAX_P;
    motor_b(motorrichtung_B);
    t_taster_lang_b = 0;
    }

void init_io(void)
    {
    setPwmFrequency(10, 2);
    setPwmFrequency(9, 2);
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
    analogWrite(Motor_A_EN, 0);
    digitalWrite(LED_B_PIN, 0);
    analogWrite(Motor_B_EN, 0);
    }

void lese_richtung(void)
    {
    motorrichtung_A = EEPROM.read(eeRichtung_A);
    motorrichtung_B = EEPROM.read(eeRichtung_B);
    if (motorrichtung_A != 1 && motorrichtung_A != 2)
	{
	motorrichtung_A = 1;
	eeprom_update_byte(eeRichtung_A, motorrichtung_A);
	}
    if (motorrichtung_B != 1 && motorrichtung_B != 2)
	{
	motorrichtung_B = 1;
	eeprom_update_byte(eeRichtung_B, motorrichtung_B);
	}
    }

void schreibe_richtung(void)
    {
    eeprom_update_byte(eeRichtung_A, motorrichtung_A);
    eeprom_update_byte(eeRichtung_B, motorrichtung_B);
    }

void aender_richtung_A(void)
    {
    if (motorrichtung_A == 1)
	{
	motorrichtung_A = 2;
	}
    else if (motorrichtung_A == 2)
	{
	motorrichtung_A = 1;
	}
    schreibe_richtung();
    }

void aender_richtung_B(void)
    {
    if (motorrichtung_B == 1)
	{
	motorrichtung_B = 2;
	}
    else if (motorrichtung_B == 2)
	{
	motorrichtung_B = 1;
	}
    schreibe_richtung();
    }

void taster_debounce(void)
    {
    if (digitalRead(F_Lose_A_PIN) == 0 && t_taster_lose_a < 255)
	{
	t_taster_lose_a++;
	}
    else if (digitalRead(F_Lose_A_PIN) && t_taster_lose_a > 0)
	{
	t_taster_lose_a--;
	}
    if (digitalRead(F_Lose_B_PIN) == 0 && t_taster_lose_b < 255)
  	{
  	t_taster_lose_b++;
  	}
      else if (digitalRead(F_Lose_B_PIN) && t_taster_lose_b > 0)
  	{
  	t_taster_lose_b--;
  	}
    if (digitalRead(F_Fest_A_PIN) == 0 && t_taster_fest_a < 255)
	{
	t_taster_fest_a++;
	}
    else if (digitalRead(F_Fest_A_PIN) && t_taster_fest_a > 0)
	{
	t_taster_fest_a--;
	}
    if (digitalRead(F_Fest_B_PIN) == 0 && t_taster_fest_b < 255)
  	{
  	t_taster_fest_b++;
  	}
      else if (digitalRead(F_Fest_B_PIN) && t_taster_fest_b > 0)
  	{
  	t_taster_fest_b--;
  	}
    if (digitalRead(Ein_Ziehen_A_PIN) == 0 && t_taster_einzieh_a < 255)
	{
	t_taster_einzieh_a++;
	}
    else if (digitalRead(Ein_Ziehen_A_PIN) && t_taster_einzieh_a > 0)
	{
	t_taster_einzieh_a--;
	}
    if (digitalRead(Ein_Ziehen_B_PIN) == 0 && t_taster_einzieh_b < 255)
  	{
  	t_taster_einzieh_b++;
  	}
      else if (digitalRead(Ein_Ziehen_B_PIN) && t_taster_einzieh_b > 0)
  	{
  	t_taster_einzieh_b--;
  	}
    if (digitalRead(Putzen_A_PIN) == 0 && t_taster_putzen_a < 255)
 	{
 	t_taster_putzen_a++;
 	}
     else if (digitalRead(Putzen_A_PIN) && t_taster_putzen_a > 0)
 	{
 	t_taster_putzen_a--;
 	}
     if (digitalRead(Putzen_B_PIN) == 0 && t_taster_putzen_b < 255)
   	{
   	t_taster_putzen_b++;
   	}
       else if (digitalRead(Putzen_B_PIN) && t_taster_putzen_b > 0)
   	{
   	t_taster_putzen_b--;
   	}
    }

//zählt jede ms die Timer hoch
void setTimer(void)
    {
    if (micros() >= t_timer)
	{
	t_p_start_a++;
	t_m_pwm_a++;
	t_led_a++;
	t_p_start_b++;
	t_m_pwm_b++;
	t_led_b++;
	if (t_taster_putzen_a >= TASTER_Debounce && status_putzen_a == 0)
	    {
	    t_taster_lang_a = t_taster_lang_a + 1;
	    }
	if (t_taster_putzen_b >= TASTER_Debounce && status_putzen_b == 0)
	    {
	    t_taster_lang_b = t_taster_lang_b + 1;
	    }
	taster_debounce();
	t_timer = t_timer + 1000;
	}
    }

void tasterAbfrage(void)
    {
    if (digitalRead(SAVE_PIN) == 0)
	{
	// Einziehen starten Motor A
	if (t_taster_einzieh_a >= TASTER_Debounce && status_putzen_a == 0)
	    {
	    set_einziehen_A();
	    }
	// Putzen starten Motor A
	if (status_putzen_a == 0 && t_taster_lang_a >= T_Taster_Lang)
	    {
	    set_putzen_A();
	    }
	// Einziehen starten Motor B
	if (t_taster_einzieh_b >= TASTER_Debounce && status_putzen_b == 0)
	    {
	    set_einziehen_B();
	    }
	// Putzen starten Motor B
	if (status_putzen_b == 0 && t_taster_lang_b >= T_Taster_Lang)
	    {
	    set_putzen_B();
	    }
	}
    // Verhindert, dass nach einem Putzvorgang direkt ein zweiter startet
    if (t_taster_einzieh_a <= 5 && t_taster_putzen_a <= 5
	    && status_putzen_a == 3 && t_p_start_a >= 200)
	{
	status_putzen_a = 0;
	}
    if (t_taster_einzieh_b <= 5 && t_taster_putzen_b <= 5
	    && status_putzen_b == 3 && t_p_start_b >= 200)
	{
	status_putzen_b = 0;
	}
    // reset des Zählers für lange Tastendrücke
    if (t_taster_putzen_a <= 5)
	{
	t_taster_lang_a = 0;
	}
    if (t_taster_putzen_b <= 5)
	{
	t_taster_lang_b = 0;
	}
    }

void check_Ende(void)
    {
    if (status_putzen_a == 4)
	{
	set_bremse_a();
	aender_richtung_A();
	digitalWrite(LED_A_PIN, 0);
	}
    if (status_putzen_a == 5)
	{
	set_bremse_a();
	digitalWrite(LED_A_PIN, 0);
	}
    if (status_putzen_a == 6)
	{
	set_bremse_a();
	digitalWrite(LED_A_PIN, 1);
	}
    if (status_putzen_b == 4)
	{
	set_bremse_b();
	aender_richtung_B();
	digitalWrite(LED_B_PIN, 0);
	}
    if (status_putzen_b == 5)
	{
	set_bremse_b();
	digitalWrite(LED_B_PIN, 0);
	}
    if (status_putzen_b == 6)
	{
	set_bremse_b();
	digitalWrite(LED_B_PIN, 1);
	}
    }

//The setup function is called once at startup of the sketch
void setup()
    {
    init_io();
    lese_richtung();
    }

// The loop function is called in an endless loop
void loop()
    {
    setTimer();
    tasterAbfrage();
    if (status_putzen_a == 1 || status_putzen_a == 2 || status_putzen_a == 7)
	{
	set_motorpower_a();
	}
    if (status_putzen_b == 1 || status_putzen_b == 2 || status_putzen_b == 7)
	{
	set_motorpower_b();
	}
    if (status_putzen_a == 1)
	{
	if (t_p_start_a >= T_MAX_P)
	    {
	    status_putzen_a = 6;
	    }
	if ((t_p_start_a >= T_MIN_P) && t_taster_fest_a >= TASTER_Debounce)
	    status_putzen_a = 4;
	if (t_led_a == LED_T_P)
	    {
	    digitalWrite(LED_A_PIN, !digitalRead(LED_A_PIN));
	    t_led_a = 0;
	    }
	if (t_taster_einzieh_a >= TASTER_Debounce)
	    {
	    status_putzen_a = 6;
	    }
	if (t_p_start_a >= T_MIN_P && t_taster_lose_a <= 5)
	    {
	    motorpower_a = 0;
	    pwmVerzoger_A = VERZOGER_E;
	    }
	}
    if (status_putzen_a == 2)
	{
	// maximale Einziehzeit erreicht
	if (t_p_start_a >= T_MAX_E)
	    {
	    status_putzen_a = 6;
	    }
	//Stopp bei drücken des Putzen Pins
	if (t_taster_putzen_a >= TASTER_Debounce)
	    {
	    status_putzen_a = 6;
	    }
	// Stopp bei erreichen des Fest-Tasters
	if (t_taster_fest_a >= TASTER_Debounce)
	    {
	    status_putzen_a = 5;
	    }
	// LED Blinken
	if (t_led_a >= LED_T_E)
	    {
	    digitalWrite(LED_A_PIN, !digitalRead(LED_A_PIN));
	    t_led_a = 0;
	    }
	if ( t_taster_lose_a <= 5)
	    {
	    motorpower_a = 0;
	    }
	}
    if (status_putzen_b == 1)
	{
	if (t_p_start_b >= T_MAX_P)
	    {
	    status_putzen_b = 6;
	    }
	if ((t_p_start_b >= T_MIN_P) && t_taster_fest_b >= TASTER_Debounce)
	    {
	    status_putzen_b = 4;
	    }
	if (t_led_b == LED_T_P)
	    {
	    digitalWrite(LED_B_PIN, !digitalRead(LED_B_PIN));
	    t_led_b = 0;
	    }
	if (t_taster_einzieh_b >= TASTER_Debounce)
	    {
	    status_putzen_b = 6;
	    }
	if (t_p_start_b >= T_MIN_P &&  t_taster_lose_b <= 5)
	    {
	    motorpower_b = 0;
	    pwmVerzoger_B = VERZOGER_E;
	    }
	}
    if (status_putzen_b == 2)
	{
	// maximale Einziehzeit erreicht
	if (t_p_start_b >= T_MAX_E)
	    {
	    status_putzen_b = 6;
	    }
	//Stopp bei drücken des Putzen Pins
	if (t_taster_putzen_b >= TASTER_Debounce)
	    {
	    status_putzen_b = 6;
	    }
	// Stopp bei erreichen des Fest-Tasters
	if (t_taster_fest_b >= TASTER_Debounce)
	    {
	    status_putzen_b = 5;
	    }
	// LED Blinken
	if (t_led_b >= LED_T_E)
	    {
	    digitalWrite(LED_B_PIN, !digitalRead(LED_B_PIN));
	    t_led_b = 0;
	    }
	if (t_p_start_b >= T_MIN_P &&  t_taster_lose_b <= 5)
	    {
	    motorpower_b = 0;
	    }
	}
    // Motor bremsen
    if (status_putzen_a == 7 && motorpower_a == 255)
	{
	status_putzen_a = 3;
	t_p_start_a = 0;
	}
    if (status_putzen_b == 7 && motorpower_b == 255)
	{
	status_putzen_b = 3;
	t_p_start_b = 0;
	}
    // Putzen beendet?
    check_Ende();
    }
