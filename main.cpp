#include <avr/io.h>

uint16_t readADC(uint8_t ch)
{
	// select adc channel (0-7)
	ch = ch & 0b00000111;	// bit masking just ensures that channel can never be > 7
	ADMUX |= ch;
	
	// start a conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1<<ADSC);
	
	// wait for it to complete: ADIF bit gets set when conversion is complete
	// ASM equiv: sbis	ADCSR, ADIF
	while (!(ADCSRA & (1<<ADIF))) {};
	
	// clear ADIF
	// From the datasheet i thought this happened automatically, but perhaps not...
	ADCSRA |= (1<<ADIF);
	
	return ADC;
}

void initADC()
{
	// set micro to use VCC with external decoupling cap as reference voltage
	ADMUX = (1<<REFS0);
	
	// set to approx 93.75kHz (with a 12 meg crystal on ousb): division factor of 128
	ADCSRA= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	
	// This could be re-thought - decided to throw port inits in here
	PORTA= 0x00;	// turn off PORTA pull-ups
	DDRA = 0x00;	// all port A inputs
	PORTB= 0x00;	// all PORTB outputs low
	DDRB = 0xFF;	// all port B outputs
	PORTC= 0xFF;	// all PORTC pull-ups on
	DDRC = 0x00;	// all port C inputs
}

void initPWM()
{
	TCCR0 |= (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS00); // x1101001

	// OC0 shares function with PB3, so it must be set to output to get a result
	DDRB |= (1<<PB3); // set OC0 to output
}

int main()
{
	uint16_t adc_result = 0x0000;
	uint8_t duty_cycle = 0x00;
	float scaled = 0.0;
	uint8_t adjust = 46; // adjust hard-coded for now - should be made adjustable by trimpot or something

	initPWM();
	initADC();
	
	while (1)
	{
		adc_result = readADC(0);
		/**********************************
		** MAGIC PID STUFF HAPPENS HERE ***
		**********************************/
		scaled = ((float)adc_result / (float)1023) * 255;
		duty_cycle = (uint8_t)scaled; // scale to 8 bits
		OCR0 = duty_cycle + adjust; // set duty cycle, 0-255 (255 = 100%)
	}
	
	return 0;
}