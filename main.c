/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Example of use of general PID implementation for AVR.
 *
 * Example of how to setup and use the general PID implementation in pid.c.
 *
 * - File:               main.c
 * - Compiler:           IAR EWAAVR 4.11A
 * - Supported devices:  All AVR devices can be used.
 * - AppNote:            AVR221 - Discrete PID controller
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr@atmel.com
 *
 * $Name$
 * $Revision: 456 $
 * $RCSfile$
 * $Date: 2006-02-16 12:46:13 +0100 (to, 16 feb 2006) $
 *****************************************************************************/

#include <inavr.h>
#include <ioavr.h>
#include "stdint.h"
#include "pid.h"

/*! \brief P, I and D parameter values
 *
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 */
//! \xrefitem todo "Todo" "Todo list"
#define K_P     1.00
//! \xrefitem todo "Todo" "Todo list"
#define K_I     0.00
//! \xrefitem todo "Todo" "Todo list"
#define K_D     0.00

/*! \brief Flags for status information
 */
struct GLOBAL_FLAGS {
  //! True when PID control loop should run one time
  uint8_t pidTimer:1;
  uint8_t dummy:7;
} gFlags = {0, 0};

//! Parameters for regulator
struct PID_DATA pidData;

/*! \brief Sampling Time Interval
 *
 * Specify the desired PID sample time interval
 * With a 8-bit counter (255 cylces to overflow), the time interval value is calculated as follows:
 * TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz] ) / 255
 */
//! \xrefitem todo "Todo" "Todo list"
#define TIME_INTERVAL   157

/*! \brief Timer interrupt to control the sampling interval
 */
#pragma vector = TIMER0_OVF_vect
__interrupt void TIMER0_OVF_ISR( void )
{
  static uint16_t i = 0;
  if(i < TIME_INTERVAL)
    i++;
  else{
    gFlags.pidTimer = TRUE;
    i = 0;
  }
}

/*! \brief Init of PID controller demo
 */
void Init(void)
{
  pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);

  // Set up timer, enable timer/counte 0 overflow interrupt
  // TCCROA doesn't exist in iom32.h so changed it.
  //TCCR0A = (1<<CS00);  
  TCCR0 = (1<<CS00);
  // Same deal with the following.  
  //TIMSK0 = (1<<TOIE0);
  TIMSK = (1<<TOIE0);
  TCNT0 = 0;
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

/*! \brief Read reference value.
 *
 * This function must return the reference value.
 * May be constant or varying
 */
int16_t Get_Reference(void)
{
  return 0;
}

/*! \brief Read system process value
 *
 * This function must return the measured data
 */
int16_t Get_Measurement(uint8_t ch)
{
  // select adc channel (0-7)
  ch = ch & 7;	// bit masking just ensures that channel can never be > 7
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
  //return 4;
}

/*! \brief Set control input to system
 *
 * Set the output from the controller as input
 * to system.
 */
void Set_Input(int16_t inputValue)
{
  uint8_t duty_cycle = 0x00;
  float scaled = 0.0;
  uint8_t adjust = 46; // adjust hard-coded for now - should be made adjustable by trimpot or something

  scaled = ((float)inputValue / (float)1023) * 255;
  duty_cycle = (uint8_t)scaled; // scale to 8 bits
  OCR0 = duty_cycle + adjust; // set duty cycle, 0-255 (255 = 100%)
}


/*! \brief Demo of PID controller
 */
void main(void)
{
  int16_t referenceValue, measurementValue, inputValue;
  Init();
  initADC();
  initPWM();
  __enable_interrupt();

  while(1){

    // Run PID calculations once every PID timer timeout
    if(gFlags.pidTimer)
    {
      referenceValue = Get_Reference();
      measurementValue = Get_Measurement(0);

      inputValue = pid_Controller(referenceValue, measurementValue, &pidData);

      Set_Input(inputValue);

      gFlags.pidTimer = FALSE;
    }
  }
}

/*! \mainpage
 * \section Intro Introduction
 * This documents data structures, functions, variables, defines, enums, and
 * typedefs in the software for application note AVR221.
 *
 * \section CI Compilation Info
 * This software was written for the IAR Embedded Workbench 4.11A.
 *
 * To make project:
 * <ol>
 * <li> Add the file main.c and pid.c to project.
 * <li> Under processor configuration, select desired Atmel AVR device.
 * <li> Enable bit definitions in I/O include files
 * <li> High optimization on speed is recommended for best performance
 * </ol>
 *
 * \section DI Device Info
 * The included source code is written for all Atmel AVR devices.
 *
 * \section TDL ToDo List
 * \todo Put in own code in:
 * \ref Get_Reference(void), \ref Get_Measurement(void) and \ref Set_Input(int16_t inputValue)
 *
 * \todo Modify the \ref K_P (P), \ref K_I (I) and \ref K_D (D) gain to adapt to your application
 * \todo Specify the sampling interval time \ref TIME_INTERVAL
 */

