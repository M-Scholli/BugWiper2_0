#include "BugWiper2_0.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <EEPROM.h>

#define Putzen_A_PIN		5
#define Ein_Ziehen_A_PIN	6
#define Putzen_B_PIN		0
#define Ein_Ziehen_B_PIN	1
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
#define eeRichtung		0

//Globale Variablen
int8_t motorrichtung;
uint8_t motorpower = 0;
uint16_t t_taster_lang = 0;	// Zeit seit rücken des Langen Tasters
uint8_t t_m_pwm_a = 0; 		//Motor PWM
uint16_t t_led_a = 0; 		//LED
uint32_t t_p_start = 0;		// T_MIN , T_MAX

/*Statusanzeige vom Putzvorgang
0 = Warten auf Tastendruck
1 = ganzer Putzvorgang
2 = Seil einziehen
3 = warten auf Taster reset
4 = putzen fertig
5 = einziehen fertig
6 = ERROR
 */
uint8_t status_putzen_a = 0;

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

void set_motorpower_a(uint8_t b) {
	analogWrite(Motor_A_EN, b);
}

void init_io(void)
    {
    pinMode(F_Fest_A_PIN, INPUT_PULLUP);
    pinMode(F_Lose_A_PIN, INPUT_PULLUP);
    pinMode(SAVE_PIN, INPUT_PULLUP);
    pinMode(Motor_A_EN, OUTPUT);
    pinMode(Motor_A_IN1, OUTPUT);
    pinMode(Motor_A_IN2, OUTPUT);
    pinMode(LED_A_PIN, OUTPUT);
    pinMode(Ein_Ziehen_A_PIN, INPUT_PULLUP);
    pinMode(Putzen_A_PIN, INPUT_PULLUP);
    digitalWrite(Motor_A_IN2, 1);
    digitalWrite(Motor_A_IN1, 1);
    digitalWrite(LED_A_PIN, 0);
    analogWrite(Motor_A_EN, 0);
    }

void lese_richtung(void) {
	motorrichtung = EEPROM.read(eeRichtung);
	if (motorrichtung != 1 && motorrichtung != 2) {
		motorrichtung = 1;
		eeprom_update_byte(eeRichtung, motorrichtung);
	}
}

void schreibe_richtung(void) {
	eeprom_update_byte(eeRichtung, motorrichtung);
}

void aender_richtung(void) {
	if (motorrichtung == 1) {
		motorrichtung = 2;
	} else if (motorrichtung == 2) {
		motorrichtung = 1;
	}
	schreibe_richtung();
}

void stop(void) {
	uint8_t t = 0;
	set_motorpower_a(BREMSE_START);
	motor_a(3);
	while (motorpower < 255) {
		t++;
		if (t == VERZOGER_B) {
			motorpower++;
			set_motorpower_a(motorpower);
			t = 0;
		}
		_delay_us(250);
	}
	motor_a(3);
	set_motorpower_a(255);
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
    t_p_start++;
    t_m_pwm_a++;
    t_led_a++;
    if (status_putzen_a == 1)
	{
	if (t_m_pwm_a == VERZOGER_P)
	    {
	    if (motorpower < EINZIEH_MAX_P)
		{
		motorpower++;
		set_motorpower_a(motorpower);
		}
	    t_m_pwm_a = 0;
	    }
	if (t_p_start >= T_MAX_P)
	    {
	    status_putzen_a = 6;
	    }
	if ((t_p_start >= T_MIN_P) && digitalRead(F_Fest_A_PIN) == 0)
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
	if (t_p_start >= T_MIN_P && digitalRead(F_Lose_A_PIN) == 0)
	    {
	    motorpower=0;
	    }
	}
    if (status_putzen_a == 2)
	{
	//langsames anfahren der Motors
	if (t_m_pwm_a == VERZOGER_E)
	    {
	    if (motorpower < EINZIEH_MAX_E)
		{
		motorpower++;
		set_motorpower_a(motorpower);
		}
	    t_m_pwm_a = 0;
	    }
	// maximale Einziehzeit erreicht
	if (t_p_start == T_MAX_E)
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
	    motorpower = 0;
	    }
	}
    if (digitalRead(SAVE_PIN) == 0)
	{
	if (digitalRead(Ein_Ziehen_A_PIN) == 0 && status_putzen_a == 0)
	    {
	    status_putzen_a = 2;
	    t_led_a = 0;
	    t_m_pwm_a = 0;
	    t_p_start = 0;
	    set_motorpower_a(motorpower = START_POWER_F);
	    if (motorrichtung == 1)
		motor_a(2);
	    else
		motor_a(1);
	    }
	if (digitalRead(Putzen_A_PIN)
		== 0&& status_putzen_a == 0 && t_taster_lang >= T_Taster_Lang)
	    {
	    status_putzen_a = 1;
	    t_m_pwm_a = 0;
	    t_led_a = 0;
	    t_p_start = 0;
	    set_motorpower_a(motorpower = START_POWER_P);
	    motor_a(motorrichtung);
	    t_taster_lang = 0;
	    }
	}
    if (digitalRead(Putzen_A_PIN) == 0 && status_putzen_a == 0)
	{
	t_taster_lang = t_taster_lang + 1;
	}
// Verhindert, dass nach einem Putzvorgang direkt ein zweiter startet
    if (digitalRead(Ein_Ziehen_A_PIN) == 1 && digitalRead(Putzen_A_PIN) == 1
	    && status_putzen_a == 3)
	{
	status_putzen_a = 0;
	delay(50);
	}
    if (digitalRead(Putzen_A_PIN) == 1)
	{
	t_taster_lang = 0;
	}
// Putzen beenden
    if (status_putzen_a == 4)
	{
	stop();
	aender_richtung();
	digitalWrite(LED_A_PIN, 0);
	status_putzen_a = 3;
	}
    if (status_putzen_a == 5)
	{
	stop();
	digitalWrite(LED_A_PIN, 0);
	status_putzen_a = 3;
	}
    if (status_putzen_a == 6)
	{
	stop();
	digitalWrite(LED_A_PIN, 1);
	status_putzen_a = 3;
	}
    delay(1);
    }
