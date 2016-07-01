// Do not remove the include below
#include "BugWiper2_0.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <EEPROM.h>
//#include <avr/eeprom.h>

#ifndef EEMEM
#define EEMEM  __attribute__ ((section (".eeprom")))
#endif

#ifndef F_CPU
#define F_CPU           16000000                   // processor clock frequency
#warning kein F_CPU definiert
#endif

#define KEY_DDR         DDRD
#define KEY_PORT        PORTD
#define KEY_PIN         PIND
#define KEY0            6	// Seil festziehen
#define KEY1            5	// Putzen starten
#define KEY2            4   //Sicherheits schalter
#define KEY3			2	//Fahrwerk
#define ALL_KEYS        (1<<KEY0 | 1<<KEY1 | 1<<KEY2)

#define REPEAT_MASK     (1<<KEY1 | 1<<KEY0)      // repeat: key1, key2
#define REPEAT_START    500                        // after 500ms
#define REPEAT_NEXT     60                        // every 200ms

#define F_FEST_DDR		DDRC
#define F_FEST_PORT		PORTC
#define F_FEST_PINS		PINC
#define F_FEST			1
#define F_FEST_PIN		PINC1

#define FAHRWERK_DDR	DDRD
#define FAHRWERK_PORT	PORTD
#define FAHRWERK_PINS	PIND
#define FAHRWERK		PD3
#define FAHRWERK_PIN	PIND3

#define F_LOSE_DDR		DDRD
#define F_LOSE_PORT		PORTD
#define F_LOSE_PINS		PIND
#define F_LOSE			4
#define F_LOSE_PIN		PIND4

#define LED1_DDR		DDRB
#define	LED1_PORT		PORTB
#define LED1			5
#define LED_T_P			50	//Zeit zum blinken
#define LED_T_F			20

#define EINZIEH_MAX_P	255	//Max Power Putzen
#define EINZIEH_MAX_F	255 //Max Motorpower f�r Festzieh mit Fahrwerk
#define EINZIEH_MAX		255 //Max Motorpower f�r Einzieh im Flug
#define EINZIEH_MIN_STOP_F 140
#define VERZOGER		10	//wie viel ms bis zum erh�hen der Motorpower um 1
#define BREMSE_START	210 //Motorpower bremsen startwert
#define VERZOGER_B		2  //Wie viel bis zum erh�hen beim Bremsen in 250�s
#define VERZOGER_P		60 //Verz�gern Putzen
#define VERZOGER_F		20
#define VERZOGER_STOP_F 1
#define T_MIN_P			18//Minimale Putzzeit[ms] = T_MIN_P * VERZOGER_P
#define T_MAX_P			9000//Maximale Putzzeit
#define T_MAX_F			5000 //Maximale festziehzeit //erh�hen der einziehzeit
#define START_POWER_P	40 //motorpower am anfang Putzen
#define START_POWER_F	70


//input 1-4
#define L298_PORT_I1	PORTB
#define L298_DDR_I1		DDRB
#define L298_I1			4

#define L298_PORT_I2	PORTB
#define L298_DDR_I2		DDRB
#define L298_I2			2

#define L298_PORT_I3
#define L298_DDR_I3
#define L298_I3

#define L298_PORT_I4
#define L298_DDR_I4
#define L298_I4

//Enable A Pin
#define L298_PORT_EA	PORTB
#define L298_DDR_EA		DDRB
#define L298_EA			PB1

//Enable B Pin
#define L298_PORT_EB
#define L298_DDR_EB
#define L298_EB

//PWM init
#define L298_PWM_A		OCR0A
#define L298_PORT_PWM_A	PORTB
#define L298_DDR_PWM_A	DDRB
#define L298_PWM_PIN_A	1
#define L298_PWM_B

//PWM config
#define L298_TCCRA_A		TCCR0A
#define L298_TCCRA_WERT_A	0x83
#define L298_TCCRB_A		TCCR0B
#define L298_TCCRB_WERT_A	0x03
#define eeRichtung		0
volatile uint16_t key_state;                // debounced and inverted key state:
// bit = 1: key pressed
volatile uint16_t key_press;                                // key press detect

volatile uint16_t key_rpt;                          // key long press and repeat

//master slave deklarieren
//soll sp�ter eine Jumperposition abgefragt werden.
uint8_t master = 1;		//1=master 0=slave
//uint8_t not_stop = 0;
//uint8_t motor_an = 0;		//0=aus
int8_t motorrichtung;
uint8_t motorpower = 0;
uint8_t fahrwerk = 0; //fahrwerkausgefahren? flug=0 boden=1 test=3
//uint8_t key = 0;
//volatile int8_t z;
//volatile int8_t sekunde = T_MAX;
/*
 ISR( TIMER1_COMPA_vect ) {	//z�hlt die Zeit runter
 if (motor_an == 1) {
 z++;
 if (z == 3) { //normal 125
 sekunde--;
 z = 0;
 if (sekunde == -1) {
 sekunde = T_MAX;
 motor_soft_a(3);
 not_stop = 1;
 motor_an = 0;
 }
 }
 }
 }

 void TIMER1_interrupt_init(void) //konfiguriert die Uhr
 {
 z = 0; //ISR-Z�hler = 0
 TCNT1 = 0; //Anfangsz�hlerstand = 0
 OCR1AL = 1; //Z�hler z�hlt bis 124: 15625Hz/124 =126
 TCCR1B = 0x1d; //CTC-Modus: Takt intern von 16 Mhz /1024 =15625Hz
 //Timer/Counter0 Compare Match Interrupt aktivieren:
 TIMSK1 |= (1 << OCIE1A);
 //sei();
 }
 */
// Timer f�r Key-Debounce

uint8_t eeprom_read_byte(int adresse)
{
  return EEPROM.read(adresse);
}

void eeprom_update_byte(int adresse, uint8_t wert)
{
  if( EEPROM.read(adresse) != wert ){
        EEPROM.write(adresse, wert);
      }}

ISR( TIMER2_OVF_vect )                            // every 10ms
{
	static uint8_t ct0, ct1;
	static uint16_t rpt;
	uint8_t i;
	TCNT2 = (uint8_t) (int16_t) -(F_CPU / 1024 * 10e-3 + 0.5); // preload for 10ms
	i = key_state ^ ~KEY_PIN;                       // key changed ?
	ct0 = ~(ct0 & i);                             // reset or count ct0
	ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
	i &= ct0 & ct1;                                 // count until roll over ?
	key_state ^= i;                               // then toggle debounced state
	key_press |= key_state & i;                     // 0->1: key press detect
	if ((key_state & REPEAT_MASK) == 0)            // check repeat function
		rpt = REPEAT_START;                          // start delay
	if (--rpt == 0) {
		rpt = REPEAT_NEXT;                            // repeat delay
		key_rpt |= key_state & REPEAT_MASK;
	}
}


void init_pwm_a() {
	L298_DDR_PWM_A |= (1 << L298_PWM_PIN_A);
	L298_TCCRA_A = L298_TCCRA_WERT_A;
	L298_TCCRB_A = L298_TCCRB_WERT_A;
	L298_PWM_A = 0;
}

// Schaltet den Motor 1= richtung 1;	2=richtung 2;	3=stopp;
void motor_a(uint8_t a) {
	switch (a) {
	case 1: {
		L298_PORT_I2 &= ~(1 << L298_I2);
		L298_PORT_I1 |= (1 << L298_I1);
		//OCR0A=255;
		/*for(int i=100;i<=254;i++)
		 {
		 L298_PWM_A=i;
		 _delay_ms(1);
		 }
		 */
	}
		break;

	case 2: {
		L298_PORT_I1 &= ~(1 << L298_I1);
		L298_PORT_I2 |= (1 << L298_I2);
		/*for(int i=100;i<=255;i++)
		 {
		 L298_PWM_A=i;
		 _delay_ms(1);
		 }
		 */
	}
		break;

	case 3: {
		//L298_PWM_A = 255;
		L298_PORT_I2 &= ~(1 << L298_I2);
		L298_PORT_I1 &= ~(1 << L298_I1);

		/*for(int i=200;i<=255;i++)
		 {
		 OCR0A=i;
		 _delay_ms(1);
		 }*/
	}
		break;
	}
}

void motor_soft_a(uint8_t a) {
	switch (a) {
	case 1: {
		L298_PORT_I2 &= ~(1 << L298_I2);
		L298_PORT_I1 |= (1 << L298_I1);
		//OCR0A=255;
		for (int i = 100; i <= 254; i++) {
			L298_PWM_A = i;
			_delay_ms(1);
		}

	}
		break;

	case 2: {
		L298_PORT_I1 &= ~(1 << L298_I1);
		L298_PORT_I2 |= (1 << L298_I2);
		for (int i = 100; i <= 255; i++) {
			L298_PWM_A = i;
			_delay_ms(1);
		}

	}
		break;

	case 3: {
		L298_PWM_A = 255;
		L298_PORT_I2 |= (1 << L298_I2);
		L298_PORT_I1 |= (1 << L298_I1);

		/*for(int i=200;i<=255;i++)
		 {
		 OCR0A=i;
		 _delay_ms(1);
		 }*/
	}
		break;
	}
}

void set_motorpower_a(uint8_t b) {
	L298_PWM_A = b;
}


void key_init(void) {
	// Configure debouncing routines
	KEY_DDR &= ~ALL_KEYS;                // configure key port for input
	KEY_PORT |= ALL_KEYS;                // and turn on pull up resistors
	TCCR2B = (1 << CS22) | (1 << CS21);         // divide by 1024
	TCNT2 = (uint8_t) (int16_t) -(F_CPU / 256 * 10e-3 + 0.5); // preload for 10ms
	TIMSK2 |= 1 << TOIE2;                   // enable timer interrupt
}
///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed. Each pressed key is reported
// only once
//
uint16_t get_key_press(uint16_t key_mask) {
	cli();
	// read and clear atomic !
	key_mask &= key_press;                          // read key(s)
	key_press ^= key_mask;                          // clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint16_t get_key_rpt(uint16_t key_mask) {
	cli();
	// read and clear atomic !
	key_mask &= key_rpt;                            // read key(s)
	key_rpt ^= key_mask;                            // clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint16_t get_key_state(uint16_t key_mask)

{
	key_mask &= key_state;
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint16_t get_key_short(uint16_t key_mask) {
	cli();
	// read key state and key press atomic !
	return get_key_press(~key_state & key_mask);
}

///////////////////////////////////////////////////////////////////
//
uint16_t get_key_long(uint16_t key_mask) {
	return get_key_press(get_key_rpt(key_mask));
}

void init_io(void) {
	F_FEST_DDR &= ~(1 << F_FEST);
	F_LOSE_DDR &= ~(1 << F_LOSE);
	F_FEST_PORT |= (1 << F_FEST);
	F_LOSE_PORT |= (1 << F_LOSE);
	LED1_DDR |= (1 << LED1);
	LED1_PORT &= ~(1 << LED1);
}

void lese_richtung(void) {
	motorrichtung = eeprom_read_byte(eeRichtung);
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

void festziehen(void) {
	uint8_t t1 = 0;
	uint8_t t3 = 0;
	uint8_t t4 = 0;
	uint16_t t2 = 0;
	uint8_t run = 1;                            //motor l�uft
	uint8_t safe = 1;
	if (!(PINC & (1 << PINC3))) {         //F_LOSE_PINS & (1 << F_LOSE_PIN)) {
		set_motorpower_a(motorpower = START_POWER_F);
		if (motorrichtung == 1)
			motor_a(2);
		else
			motor_a(1);
		while (run == 1) {
			t1++;
			t4++;
			//langsames anfahren der Motors
			if (t4 == VERZOGER_F) {
				if (motorpower < EINZIEH_MAX_F) {
					motorpower++;
					set_motorpower_a(motorpower);
				}
				t4 = 0;
			}

			if (t1 == 10) {
				t2++;
				// maximale Einziehzeit erreicht
				if (t2 == T_MAX_F) {
					safe = 0;
					run = 0;
				}
				// Stopp bei erreichen des Fest-Tasters
				if ((PINC & (1 << PINC5)))
					if (!(F_FEST_PINS & (1 << F_FEST_PIN)))
						run = 0;
				// LED Blinken
				t3++;

				if (t3 == LED_T_P) {
					LED1_PORT ^= (1 << LED1);
					t3 = 0;
				}
				t1 = 0;

				if (get_key_press(1 << KEY1))
					run = 0;
			}
			_delay_ms(1);
		}
		//set_motorpower_a(255);
		//_delay_ms(300);
		/*run = 1;
		 t4 = 0;
		 motorpower = 255;
		 while (run == 1) {
		 t4++;
		 if (t4 == VERZOGER_STOP_F) {
		 if (motorpower > EINZIEH_MIN_STOP_F) {
		 motorpower--;
		 set_motorpower_a(motorpower);
		 } else {
		 run = 0;
		 }
		 t4 = 0;
		 }
		 _delay_ms(1);
		 }*/
		stop();
		LED1_PORT &= ~(1 << LED1);
		get_key_press(1 << KEY0);
		get_key_press(1 << KEY1);
		//get_key_press(1 << KEY2);
		if (safe == 1) {
			LED1_PORT &= ~(1 << LED1);
		} else
			LED1_PORT |= (1 << LED1);
	}
}

void putzen(void) {
	uint8_t t1 = 0;
	uint8_t t4 = 0;
	uint8_t t5 = 0;
	uint16_t t2 = 0;
	uint32_t t3 = 0;
	uint8_t run = 1;                            //motor l�uft
	uint8_t safe = 1;
	if (!(PINC & (1 << PINC3))) {
		set_motorpower_a(motorpower = START_POWER_P);
		motor_a(motorrichtung);
		while (run == 1) {
			t1++;
			t4++;
			if (t4 == VERZOGER_P) {
				if (motorpower < EINZIEH_MAX_P) {
					motorpower++;
					set_motorpower_a(motorpower);
				}
				t4 = 0;
			}
			if (t1 == 10) {
				t3++;
				if (t3 == T_MAX_P) {
					run = 0;
					safe = 0;
				}
				if (t2 < T_MIN_P)
					t2++;
				if ((t2 == T_MIN_P) && !(F_FEST_PINS & (1 << F_FEST_PIN)))
					run = 0;
				t5++;
				if (t5 == LED_T_P) {
					LED1_PORT ^= (1 << LED1);
					t5 = 0;
				}
				t1 = 0;
			}
			if ((F_FEST_PINS & (1 << F_FEST_PIN)))
				t2 = (T_MIN_P - 1);
			if (get_key_press(1 << KEY0)) {
				run = 0;
				safe = 0;
			}
			_delay_ms(1);
		}
		//_delay_ms(200);
		/*
		 set_motorpower_a(255);
		 _delay_ms(300);
		 run = 1;
		 t4 = 0;
		 motorpower = 255;
		 while (run == 1) {
		 t4++;
		 if (t4 == VERZOGER_STOP_F) {
		 if (motorpower > EINZIEH_MIN_STOP_F) {
		 motorpower--;
		 set_motorpower_a(motorpower);
		 } else {
		 run = 0;
		 }
		 t4 = 0;
		 }
		 _delay_ms(1);
		 }
		 */

		stop();
		get_key_press(1 << KEY0);
		get_key_press(1 << KEY1);
		//get_key_press(1 << KEY2);
		if (safe == 1) {
			aender_richtung();
			LED1_PORT &= ~(1 << LED1);
		} else
			LED1_PORT |= (1 << LED1);
	}
}

//The setup function is called once at startup of the sketch
void setup()
{
	init_pwm_a();
	//TIMER1_interrupt_init();
	key_init();
	init_io();
	lese_richtung();
	sei();
	stop();}

// The loop function is called in an endless loop
void loop()
{
	lese_richtung();
	//if (fahrwerk == 0) {
	if (get_key_short(1 << KEY0)) {
		festziehen();
	}
	if (get_key_long(1 << KEY1)) {
		putzen();
	}
	//get_key_press(1 << KEY2);
	//}
	/* else if (fahrwerk == 1) {
	 if (get_key_long(1 << KEY0)) {
	 festziehen();
	 }
	 }*/
	_delay_ms(5);}
