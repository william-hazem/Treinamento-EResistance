/*
 * EResistance.c
 *
 * Created: 03/10/2021 12:44:20
 * Last Update: 16/10/2021 23:28
 * Author : William Henrique
 */ 

#include <avr/io.h>
#define F_CPU 16e6
#include <util/delay.h>			
#include <avr/interrupt.h>

#include <stdio.h>					/// sprintf: int to char
#include <math.h>					/// random numbers

#include "nokia5110.h"				/// Nokia display controller 


#define RADIUS2 30e-2				/// Wheel Radius * 2
#define PI 3.14159265359			/// PI constant

int ofc = 0;						// timer overflow counter (milli secs)
int motor_rot = 0, rot_vel = 0;		// motors rotations
int32_t temp = 0, cell = 0, light;  // readable adc values
uint32_t time_ms = 0ul;

/* @brief Get an digit from an 8bit integer
 * @params number an 8bit integer (0 to 255)
 * @params n nth digit index that you want to separate (0 to 2)
 * @returns the a number between 0 and 9
 */
short get_digit(short number, short n)
{
    // fast 10-th powers 
    static int power10[] = {1, 1e1, 1e2, 1e3};   
	number = number % power10[n+1];
    if(n > 0) number = number / power10[n];
    return number; 
}

/* @brief translocate bits using a pattern
 * @params bits 8bit number
 * @params mapper is the new pattern
 * @returns translocated bits
 */
uint8_t mapping4b(const short bits, const short mapper)
{	
	int it = 0;
	short out = 0;
	for(int i = 0; i < 4 && it < 8; i++)
	{
		while(!(mapper & (1 << it))) it++; //find ones positions
		short mask = bits & (1 << i);	   // get only i-th bit if it's hight state
		if(mask)
			out = out | (1 << it);		   // translocate then
		it++;									
	}	
	return out;
}

/* @brief turn on an especific display
 * @params digit to be showed
 * @params port an adress to port that output the 4to8 decoder
 * @params adr current enable display
 * @returns void
 */
void display7seg(const uint8_t digit, int * port, const uint8_t adr) {
	(*port) &= ~adr;						//light off leds
	(*port) |= mapping4b(digit, adr);		//light on leds
	
}

/* @brief milliseconds counter callback
 */
ISR(TIMER1_COMPA_vect)
{
	time_ms += 1;
}

/* @brief car panel button interruption callback
 */
ISR(INT1_vect)
{
	PORTD ^= 0b10000000;
}

/* @brief rotation interruption callback
 */
ISR(INT0_vect) // handler int0 envs
{
	motor_rot++;
	rot_vel++;
}

/* @brief ADC signal read callback
 */
ISR(ADC_vect)
{
	static uint8_t current = 0;
	float adc = ADC;	
	if(current == 0) // read temp
	{
		ADMUX = 0b00000100;	  // to read power cells using aref - ch4
		double res = 95.5*adc + 20; // false calculation
		temp = res*1.1/1023;
		current++;
	}
	else if(current == 1)// read power cells
	{
		ADMUX = 0b00000011;	// to read light using aref - ch3
		cell = adc / 1023 * 100;
		current++;
	}
	else
	{
		ADMUX = 0b11000101;	// to read temp using 1.1ref - ch5
		light = (1023000/adc - 1000);
		// changes duty circle based in light %
		if(light > 500)
			OCR0A = 0;
		else if(light > 300)
			OCR0A = 125;
		else
			OCR0A = 255;
		current = 0;
	}
	
}


/* @brief Calculate mean velocity
 * @params detalt delta time
 * @returns mean velocity
 */
float vmed(uint8_t deltat) 
{
	float RPM = rot_vel / deltat;
	float VEL = RPM * PI * RADIUS2;
	rot_vel = 0;
	return VEL * 3.6; //result in KM/H
}

/* @brief main function
 * @returns 0 if completed run
 */
int main(void)
{
	/* SETUP */

	DDRB  = 0b00000001; /// OUT: B0
	DDRC  = 0b01000111; /// OUT: C0 C1 C2 C6
	DDRD  = 0b11010011;	/// OUT: D7 D1 D0 D4 D6-PWM
	PORTD = 0b00001000; /// enable D3 with pullup resistor
	uint8_t led7seg = 0b01000111;
	uint8_t led7enb[] = {(1<<0), (1<<1), (1<<4)};
	
	// TIMER - T1 8bit
	TCCR1A = 0b00000010; // ctc mode
	TCCR1B = 0b00000011; // prescale = 64
	OCR1A = 249;		 // OCRA compare value   
	TIMSK1 = 0b00000010; // enable compare overflow
	
	// PWM TIMER - T0 bit
	TCCR0A = 0b10100011;		// set fast PWM and not inverted chA chB
	TCCR0B = 0b00000011;		// not force compare regA and regB, enable prescale=64
	OCR0A = 200;			// set 0 -> duty = 0;
	// ADC
	ADMUX = 0b11000100;  // ref = 1.1v - PORT ch0-5
	ADCSRA = 0b11101111; // ADEN, ADCS, ADATE, ADIE & ADPS 
	ADCSRB = 0x0;		 // 
	DIDR0 = ~(0x4 |0x8 | 0x03);		 // enable only C4, C5, C3 as AD input 
	
	// extern interrupt setup
	EICRA = 0b00001111; // D2: int0 D3:int1
	EIMSK = 0b00000011; // enable int0 e int2
	
	// enable interruptions
	sei();
	
	nokia_lcd_init();  /// uses D0 D1 D3 D4 D5
	char str_rpm[16] = "RPM: 0", str_Vel[16] = "VEL: 0",
		str_cell[16] = "CELL: 0", str_temp[16] = "TEMP: 0";
    
	uint8_t display_number = 0;
	
	while (1) 
    {
		nokia_lcd_clear();			/// clear lcd buffer
		
		uint8_t current_digit = 0;	/// digit index
		while(current_digit < 3)
		{
			PORTD &= ~((1<<0) | (1<<1) | (1<<4));								//enable the first display											
			PORTD |= led7enb[current_digit];									// to visualize the change on SumuIde
			display7seg(get_digit(display_number, current_digit++), &PORTC, led7seg);		//display the first digit
			_delay_ms(10);	
		}
		
		static uint32_t time_1000ms = 0;
		if((time_ms - time_1000ms) > 1000)
		{
			sprintf(str_rpm, "RPM: %d rot/m", motor_rot);
			sprintf(str_cell, "CELL: %ld pc", cell);
			motor_rot = 0;
			time_1000ms = time_ms;
		}
		
		static uint32_t time_2000ms = 0;
		if((time_ms - time_2000ms) > 2000)
		{
			sprintf(str_temp, "TEMP: %ld C", temp);
			if(temp > 80)
				PORTB |= 0x1;
			else
				PORTB &= ~0x1;
			time_2000ms = time_ms;
		}
		
		static uint32_t time_5000ms = 0;
		if((time_ms - time_5000ms) > 5000) 
		{
			display_number = (uint8_t) vmed(5);
			
			sprintf(str_Vel, "VEL %d", display_number);
			
			time_5000ms = time_ms;
		}
		
		/* Update lcd display */
		nokia_lcd_set_cursor(1, 1);
		nokia_lcd_write_string(str_rpm, 1);
		nokia_lcd_set_cursor(1, 11);
		nokia_lcd_write_string(str_Vel , 1);
		nokia_lcd_set_cursor(1, 21);
		nokia_lcd_write_string(str_cell, 1);
		nokia_lcd_set_cursor(1, 31);
		nokia_lcd_write_string(str_temp, 1);
		nokia_lcd_render();
		
    }
	return 0;
}