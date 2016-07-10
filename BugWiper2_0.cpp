#include "BugWiper2_0.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <EEPROM.h>

#define Putzen_A_PIN		5
#define Ein_Ziehen_A_PIN	6
#define Putzen_B_PIN		2
#define Ein_Ziehen_B_PIN	3
#define F_Fest_A_PIN		A1
#define F_Lose_A_PIN		A2
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
#define Motor_B_IN1		11
#define Motor_B_IN2		12
#define Motor_B_EN		10 	//PWM Pin
// Kalibrierung des Putzvorganges
#define T_Taster_Lang		2000    // Zeit in ms für langen Tastendruck
#define EINZIEH_MAX_P		255	//Max Power Putzen
#define EINZIEH_MAX_E		255	//Max Motorpower beim Einziehen
#define EINZIEH_MAX_GND		255 	//Max Motorpower am Boden
#define BREMSE_START		210 	//Motorpower bremsen startwert
#define VERZOGER_B		2  	//Wie viel bis zum erh�hen beim Bremsen in 250�s
#define VERZOGER_P		60 	//Verz�gern Putzen
#define VERZOGER_E		20	//Verzögern Einziehen
#define T_MIN_P			300	//Minimale Putzzeit[ms]
#define T_MAX_P			90000	//Maximale Putzzeit
#define T_MAX_E			50000 	//Maximale festziehzeit //erh�hen der einziehzeit
#define START_POWER_P		40 	//Motorpower bei losfahren Putzen
#define START_POWER_F		70

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


// der Putzvorgang am Boden soll unterdrückt werden

void eeprom_update_byte(int adresse, uint8_t wert)
    {
    if (EEPROM.read(adresse) != wert)
	{
	EEPROM.write(adresse, wert);
	}
    }

// Schaltet den Motor 1= richtung 1;	2=richtung 2;	3=stopp;
void motor_a(uint8_t a) {
	switch (a) {
	case 1: {
		digitalWrite(Motor_A_IN2, 0) ;
		digitalWrite(Motor_A_IN1, 1) ;

	}
		break;

	case 2: {
		digitalWrite(Motor_A_IN1, 0) ;
		digitalWrite(Motor_A_IN2, 1) ;

	}
		break;

	case 3: {
		digitalWrite(Motor_A_IN2, 0) ;
		digitalWrite(Motor_A_IN1, 0) ;
	}
		break;
	}
}

void motor_b(uint8_t a) {
	switch (a) {
	case 1: {
		digitalWrite(Motor_B_IN2, 0) ;
		digitalWrite(Motor_B_IN1, 1) ;

	}
		break;

	case 2: {
		digitalWrite(Motor_B_IN1, 0) ;
		digitalWrite(Motor_B_IN2, 1) ;

	}
		break;

	case 3: {
		digitalWrite(Motor_B_IN2, 0) ;
		digitalWrite(Motor_B_IN1, 0) ;
	}
		break;
	}
}

void set_motorpower_a(uint8_t b) {
	analogWrite(Motor_A_EN, b);
}

void set_motorpower_b(uint8_t b) {
	analogWrite(Motor_B_EN, b);
}

void init_io(void)
    {
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

//The setup function is called once at startup of the sketch
void setup()
    {
    init_io();
    lese_richtung();
    }

// The loop function is called in an endless loop
void loop()
    {
    t_p_start_a++;
    t_m_pwm_a++;
    t_led_a++;
    t_p_start_b++;
    t_m_pwm_b++;
    t_led_b++;
    if (status_putzen_a == 1)
	{
	if (t_m_pwm_a == VERZOGER_P)
	    {
	    if (motorpower_a < EINZIEH_MAX_P)
		{
		motorpower_a++;
		set_motorpower_a(motorpower_a);
		}
	    t_m_pwm_a = 0;
	    }
	if (t_p_start_a >= T_MAX_P)
	    {
	    status_putzen_a = 6;
	    }
	if ((t_p_start_a >= T_MIN_P) && digitalRead(F_Fest_A_PIN) == 0)
	    status_putzen_a = 4;
	if (t_led_a == LED_T_P)
	    {
	    digitalWrite(LED_A_PIN, !digitalRead(LED_A_PIN));
	    t_led_a = 0;
	    }
	if (digitalRead(Ein_Ziehen_A_PIN) == 0)
	    {
	    status_putzen_a = 6;
	    }
	if (t_p_start_a >= T_MIN_P && digitalRead(F_Lose_A_PIN) == 0)
	    {
	    motorpower_a = 0;
	    }
	}
    if (status_putzen_a == 2)
	{
	//langsames anfahren der Motors
	if (t_m_pwm_a == VERZOGER_E)
	    {
	    if (motorpower_a < EINZIEH_MAX_E)
		{
		motorpower_a++;
		set_motorpower_a(motorpower_a);
		}
	    t_m_pwm_a = 0;
	    }
	// maximale Einziehzeit erreicht
	if (t_p_start_a == T_MAX_E)
	    {
	    status_putzen_a = 6;
	    }
	//Stopp bei drücken des Putzen Pins
	if (digitalRead(Putzen_A_PIN) == 0)
	    {
	    status_putzen_a = 5;
	    }
	// Stopp bei erreichen des Fest-Tasters
	if (digitalRead(F_Fest_A_PIN) == 0)
	    {
	    status_putzen_a = 5;
	    }
	// LED Blinken
	if (t_led_a >= LED_T_E)
	    {
	    digitalWrite(LED_A_PIN, !digitalRead(LED_A_PIN));
	    t_led_a = 0;
	    }
	if (digitalRead(F_Lose_A_PIN) == 0)
	    {
	    motorpower_a = 0;
	    }
	}
    if (status_putzen_b == 1)
	{
	if (t_m_pwm_b == VERZOGER_P)
	    {
	    if (motorpower_b < EINZIEH_MAX_P)
		{
		motorpower_b++;
		set_motorpower_b(motorpower_b);
		}
	    t_m_pwm_b = 0;
	    }
	if (t_p_start_b >= T_MAX_P)
	    {
	    status_putzen_b = 6;
	    }
	if ((t_p_start_b >= T_MIN_P) && digitalRead(F_Fest_B_PIN) == 0)
	    status_putzen_b = 4;
	if (t_led_b == LED_T_P)
	    {
	    digitalWrite(LED_B_PIN, !digitalRead(LED_B_PIN));
	    t_led_b = 0;
	    }
	if (digitalRead(Ein_Ziehen_B_PIN) == 0)
	    {
	    status_putzen_b = 6;
	    }
	if (t_p_start_b >= T_MIN_P && digitalRead(F_Lose_B_PIN) == 0)
	    {
	    motorpower_b = 0;
	    }
	}
    if (status_putzen_b == 2)
	{
	//langsames anfahren der Motors
	if (t_m_pwm_b == VERZOGER_E)
	    {
	    if (motorpower_b < EINZIEH_MAX_E)
		{
		motorpower_b++;
		set_motorpower_b(motorpower_b);
		}
	    t_m_pwm_b = 0;
	    }
	// maximale Einziehzeit erreicht
	if (t_p_start_b == T_MAX_E)
	    {
	    status_putzen_b = 6;
	    }
	//Stopp bei drücken des Putzen Pins
	if (digitalRead(Putzen_B_PIN) == 0)
	    {
	    status_putzen_b = 5;
	    }
	// Stopp bei erreichen des Fest-Tasters
	if (digitalRead(F_Fest_B_PIN) == 0)
	    {
	    status_putzen_b = 5;
	    }
	// LED Blinken
	if (t_led_b >= LED_T_E)
	    {
	    digitalWrite(LED_B_PIN, !digitalRead(LED_B_PIN));
	    t_led_b = 0;
	    }
	if (digitalRead(F_Lose_B_PIN) == 0)
	    {
	    motorpower_b = 0;
	    }
	}
    if (digitalRead(SAVE_PIN) == 0)
	{
// Einziehen starten Motor A
	if (digitalRead(Ein_Ziehen_A_PIN) == 0 && status_putzen_a == 0)
	    {
	    status_putzen_a = 2;
	    t_led_a = 0;
	    t_m_pwm_a = 0;
	    t_p_start_a = 0;
	    set_motorpower_a(motorpower_a = START_POWER_F);
	    if (motorrichtung_A == 1)
		motor_a(2);
	    else
		motor_a(1);
	    }
// Putzen starten Motor A
	if (digitalRead(Putzen_A_PIN)
		== 0&& status_putzen_a == 0 && t_taster_lang_a >= T_Taster_Lang)
	    {
	    status_putzen_a = 1;
	    t_m_pwm_a = 0;
	    t_led_a = 0;
	    t_p_start_a = 0;
	    set_motorpower_a(motorpower_a = START_POWER_P);
	    motor_a(motorrichtung_A);
	    t_taster_lang_a = 0;
	    }
// Einziehen starten Motor B
	if (digitalRead(Ein_Ziehen_B_PIN) == 0 && status_putzen_b == 0)
	    {
	    status_putzen_b = 2;
	    t_led_b = 0;
	    t_m_pwm_b = 0;
	    t_p_start_b = 0;
	    set_motorpower_b(motorpower_b = START_POWER_F);
	    if (motorrichtung_B == 1)
		motor_b(2);
	    else
		motor_b(1);
	    }
// Putzen starten Motor B
	if (digitalRead(Putzen_B_PIN)
		== 0&& status_putzen_b == 0 && t_taster_lang_b >= T_Taster_Lang)
	    {
	    status_putzen_b = 1;
	    t_m_pwm_b = 0;
	    t_led_b = 0;
	    t_p_start_b = 0;
	    set_motorpower_b(motorpower_b = START_POWER_P);
	    motor_b(motorrichtung_B);
	    t_taster_lang_b = 0;
	    }
	}
    if (digitalRead(Putzen_A_PIN) == 0 && status_putzen_a == 0)
	{
	t_taster_lang_a = t_taster_lang_a + 1;
	}
    if (digitalRead(Putzen_B_PIN) == 0 && status_putzen_b == 0)
	{
	t_taster_lang_b = t_taster_lang_b + 1;
	}
// Verhindert, dass nach einem Putzvorgang direkt ein zweiter startet
    if (digitalRead(Ein_Ziehen_A_PIN) == 1 && digitalRead(Putzen_A_PIN) == 1
	    && status_putzen_a == 3)
	{
	status_putzen_a = 0;
	delay(50);
	}
    if (digitalRead(Ein_Ziehen_B_PIN) == 1 && digitalRead(Putzen_B_PIN) == 1
	    && status_putzen_b == 3)
	{
	status_putzen_b = 0;
	delay(50);
	}
// reset des Zählers für lange Tastendrücke
    if (digitalRead(Putzen_A_PIN) == 1)
	{
	t_taster_lang_a = 0;
	}
    if (digitalRead(Putzen_B_PIN) == 1)
	{
	t_taster_lang_b = 0;
	}
// Putzen beenden
    if (status_putzen_a == 4)
	{
	t_m_pwm_a = 0;
	set_motorpower_a(BREMSE_START);
	motor_a(3);
	aender_richtung_A();
	digitalWrite(LED_A_PIN, 0);
	status_putzen_a = 7;
	}
    if (status_putzen_a == 5)
	{
	t_m_pwm_a = 0;
	set_motorpower_a(BREMSE_START);
	motor_a(3);
	digitalWrite(LED_A_PIN, 0);
	status_putzen_a = 7;
	}
    if (status_putzen_a == 6)
	{
	t_m_pwm_a = 0;
	set_motorpower_a(BREMSE_START);
	motor_a(3);
	digitalWrite(LED_A_PIN, 1);
	status_putzen_a = 7;
	}
    if (status_putzen_b == 4)
	{
	t_m_pwm_b = 0;
	set_motorpower_b(BREMSE_START);
	motor_b(3);
	aender_richtung_B();
	digitalWrite(LED_B_PIN, 0);
	status_putzen_b = 7;
	}
    if (status_putzen_b == 5)
	{
	t_m_pwm_b = 0;
	set_motorpower_b(BREMSE_START);
	motor_b(3);
	digitalWrite(LED_B_PIN, 0);
	status_putzen_b = 7;
	}
    if (status_putzen_b == 6)
	{
	t_m_pwm_b = 0;
	set_motorpower_b(BREMSE_START);
	motor_b(3);
	digitalWrite(LED_B_PIN, 1);
	status_putzen_b = 7;
	}
    // Motor bremsen
    if (status_putzen_a == 7 && t_m_pwm_a >= VERZOGER_B)
	{
	motorpower_a++;
	set_motorpower_a(motorpower_a);
	t_m_pwm_a = 0;
	}
    if (motorpower_a == 255 && status_putzen_a == 7)
	{
	motor_a(3);
	set_motorpower_a(255);
	status_putzen_a = 3;
	}
    if (status_putzen_b == 7 && t_m_pwm_b >= VERZOGER_B)
	{
	motorpower_b++;
	set_motorpower_b(motorpower_b);
	t_m_pwm_b = 0;
	}
    if (motorpower_b == 255 && status_putzen_b == 7)
	{
	motor_b(3);
	set_motorpower_b(255);
	status_putzen_b = 3;
	}
    delay(1);
    }
