#include "BugWiper2_0.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <EEPROM.h>

#define Putzen_PIN		5
#define Ein_Ziehen_PIN		6
#define F_FEST_PIN		A1
#define F_Lose_PIN		A2
#define SAVE_PIN		4	// Sicherheitsschalter deaktiviert BugWiper
#define T_Taster_Lang		2000    // Zeit in ms für langen Tastendruck
#define Time_Schritt		5	// Zeit in ms die für das durchlaufen des Main Loops benötigt wird.
//Status LED
#define LED_PIN			13
#define LED_T_P			500	//Zeit zum blinken
#define LED_T_E			250
//Motor Pins
#define Motor_I1		10
#define Motor_I2		11
#define Motor_EN		9 	//PWM Pin
// Kalibrierung des Putzvorganges
#define EINZIEH_MAX_P		255	//Max Power Putzen
#define EINZIEH_MAX_E		255	//Max Motorpower f�r Festzieh mit Fahrwerk
#define EINZIEH_MAX		255 	//Max Motorpower f�r Einzieh im Flug
#define EINZIEH_MIN_STOP_F 	140
#define BREMSE_START		210 	//Motorpower bremsen startwert
#define VERZOGER_B		2  	//Wie viel bis zum erh�hen beim Bremsen in 250�s
#define VERZOGER_P		60 	//Verz�gern Putzen
#define VERZOGER_F		20
#define T_MIN_P			300	//Minimale Putzzeit[ms]
#define T_MAX_P			90000	//Maximale Putzzeit
#define T_MAX_E			50000 	//Maximale festziehzeit //erh�hen der einziehzeit
#define START_POWER_P		40 	//motorpower am anfang Putzen
#define START_POWER_F		70

//EEPROM Speicherbereich
#define eeRichtung		0

//Globale Variablen
int8_t motorrichtung;
uint8_t motorpower = 0;
uint16_t t_taster_lang = 0;	// Zeit seit drücken des Langen Tasters
uint8_t taster_reset = 0;

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
		digitalWrite(Motor_I2, 0) ;
		digitalWrite(Motor_I1, 1) ;

	}
		break;

	case 2: {
		digitalWrite(Motor_I1, 0) ;
		digitalWrite(Motor_I2, 1) ;

	}
		break;

	case 3: {
		digitalWrite(Motor_I2, 0) ;
		digitalWrite(Motor_I1, 0) ;
	}
		break;
	}
}

void set_motorpower_a(uint8_t b) {
	analogWrite(Motor_EN, b);
}

void init_io(void)
    {
    pinMode(F_FEST_PIN, INPUT_PULLUP);
    pinMode(F_Lose_PIN, INPUT_PULLUP);
    pinMode(SAVE_PIN, INPUT_PULLUP);
    pinMode(Motor_EN, OUTPUT);
    pinMode(Motor_I1, OUTPUT);
    pinMode(Motor_I2, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(Ein_Ziehen_PIN, INPUT_PULLUP);
    pinMode(Putzen_PIN, INPUT_PULLUP);
    digitalWrite(Motor_I2, 1);
    digitalWrite(Motor_I1, 1);
    digitalWrite(LED_PIN, 0);
    analogWrite(Motor_EN, 0);
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

void festziehen(void)
    {
    uint16_t t_led_a = 0;	//LED timer
    uint8_t t_m_pwm_a = 0;	//PWM erhöhungs Timer
    uint32_t t_p_start = 0;	// Maximale Motor Zeit
    uint8_t run = 1;	//motor läuft
    uint8_t safe = 1;
    set_motorpower_a(motorpower = START_POWER_F);
    if (motorrichtung == 1)
	motor_a(2);
    else
	motor_a(1);
    while (run == 1)
	{
	t_p_start++;
	t_led_a++;
	t_m_pwm_a++;
	//langsames anfahren der Motors
	if (t_m_pwm_a == VERZOGER_F)
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
	    safe = 0;
	    run = 0;
	    }
	//Stopp bei drücken des Putzen Pins
	if (digitalRead(Putzen_PIN) == 0)
	    {
	    run = 0;
	    }
	// Stopp bei erreichen des Fest-Tasters
	if (digitalRead(F_FEST_PIN) == 0)
	    {
	    run = 0;
	    }
	// LED Blinken
	if (t_led_a >= LED_T_E)
	    {
	    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
	    t_led_a = 0;
	    }
	if (digitalRead(F_Lose_PIN) == 0)
	    {
	    motorpower = 0;
	    }
	delay(1);
	}
    stop();
    digitalWrite(LED_PIN, 0);
    if (safe == 1)
	{
	digitalWrite(LED_PIN, 0);
	}
    else
	digitalWrite(LED_PIN, 1);
    }

void putzen(void)
    {
    uint8_t t_m_pwm_a = 0;  //Motor PWM
    uint16_t t_led_a = 0; //LED
    uint32_t t_p_start = 0; // T_MIN , T_MAX
    uint8_t run = 1;                            //motor l�uft
    uint8_t safe = 1;
    set_motorpower_a(motorpower = START_POWER_P);
    motor_a(motorrichtung);
    while (run == 1)
	{
	t_p_start++;
	t_m_pwm_a++;
	t_led_a++;
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
	    run = 0;
	    safe = 0;
	    }
	if ((t_p_start >= T_MIN_P) && digitalRead(F_FEST_PIN) == 0)
	    run = 0;
	if (t_led_a == LED_T_P)
	    {
	    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
	    t_led_a = 0;
	    }
	if (digitalRead(Ein_Ziehen_PIN) == 0)
	    {
	    run = 0;
	    safe = 0;
	    }
	if (t_p_start >= T_MIN_P && digitalRead(F_Lose_PIN) == 0)
	    {
	    motorpower=0;
	    }
	delay(1);
	}
    stop();
    if (safe == 1)
	{
	aender_richtung();
	digitalWrite(LED_PIN, 0);
	}
    else
	digitalWrite(LED_PIN, 1);
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
    if (digitalRead(SAVE_PIN) == 0)
	{
	if (digitalRead(Ein_Ziehen_PIN) == 0 && taster_reset == 0)
	    {
	    festziehen();
	    taster_reset = 1;
	    }
	if (digitalRead(Putzen_PIN)
		== 0&& taster_reset == 0 && t_taster_lang >= T_Taster_Lang)
	    {
	    putzen();
	    taster_reset = 1;
	    t_taster_lang = 0;
	    }
	if (digitalRead(Putzen_PIN) == 0 && taster_reset == 0)
	    {
	    t_taster_lang = t_taster_lang + Time_Schritt;
	    }
	}
    // Verhindert das nach einem Putzvorgang direkt ein zweiter startet
    if (digitalRead(Ein_Ziehen_PIN) == 1 && digitalRead(Putzen_PIN) == 1)
	{
	taster_reset = 0;
	delay(100);
	}
    if (digitalRead(Putzen_PIN) == 1)
	{
	t_taster_lang = 0;
	}
    delay(Time_Schritt - 1);
    }
