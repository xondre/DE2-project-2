/* Defines -----------------------------------------------------------*/

#define DT PD3

#ifndef F_CPU
# define F_CPU 16000000 // CPU frequency in Hz required for delay funcs
#endif

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions

/*  Global variables         */



/* Function definitions ----------------------------------------------*/

/************************************************************************
 * Function: main
 * Purpose: 
 * Arguments: none
 * Returns: 0 if it ends successfully
 ************************************************************************/
int main(void)
{ 
  //configure pins PB1,PB2 as outputs 
  GPIO_mode_output(&DDRB, PB1);
  GPIO_mode_output(&DDRB, PB2);
  GPIO_mode_input_nopull(&DDRD, DT);

  //set OC1A(pin B1) for non-inverting output
  TCCR1A |= (1<<COM1A1); TCCR1A &= ~(1<<COM1A0);
  //set OC1B(pin B2) for non-inverting output
  TCCR1A |= (1<<COM1B1); TCCR1A &= ~(1<<COM1B0);

  //fPWM = 50 Hz, TPWM = 20 ms, prescaler = 8
  //ICR1 = 39999;

  //fPWM = 250 Hz, TPWM = 4 ms, prescaler = 1
  ICR1 = 63999;

  //fast PWM, ICR1 as TOP
  TCCR1A |= (1<<WGM11); TCCR1A &= ~(1<<WGM10);
  TCCR1B |= (1<<WGM13) | (1<<WGM12);
  
  //prescaler set to 1 = 001
  TCCR1B &= ~((1<<CS12) | (1<<CS11)); TCCR1B |= (1<<CS10);
  
  //prescaler set to 8 = 010
  //TCCR1B &= ~((1<<CS12) | (1<<CS10)); TCCR1B |= (1<<CS11);
  

  //set OC1A,OC1B value to compare with timer1, from <0;39999>
  //OCR1A = 2000;
  //OCR1B = 4000;

  //set OC1A,OC1B value to compare with timer1, from <0;63999>
  OCR1A = 24000;
  OCR1B = 24000;

  // Configure Analog-to-Digital Convertion unit
  // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
  ADMUX |= (1<<REFS0);  //setting REFS0 to 1
  ADMUX &= ~(1<<REFS1); //setting REFS1 to 0
  // Enable ADC module
  ADCSRA |= (1<<ADEN);
  // Enable conversion complete interrupt
  ADCSRA |= (1<<ADIE);
  // Set clock prescaler to 128
  ADCSRA |= ((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2));
  // Select input channel ADC0 for x axis
  ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));

  //Configure external interrupt INT0 for sensing rising edge
  EICRA |= ((1<<ISC01) | (1<<ISC00));
  //enable external interrupt INT0
  EIMSK |= (1<<INT0);
  //set timer2 overflow for 2 ms and enable timer2 overflow interrupt
  TIM2_overflow_16ms();
  TIM2_overflow_interrupt_enable();

  //global interrupt enable 
  sei();

  // Infinite loop
  while (1)
  { 
      
  }    

  return 0;   //this point is never reached
}

/* Interrupt service routines ----------------------------------------*/

ISR(TIMER2_OVF_vect)
{
  //every ~100 ms convert value on joystick axis x 
  static uint8_t counter = 0;

  if(counter < 6)
  { 
    counter++;
  }
  else
  { 
    counter = 0;
    // Start ADC conversion
    ADCSRA |= (1<<ADSC);
  }

}

//for encoder which controls PWM on pin B1
ISR(INT0_vect)
{
  if(GPIO_read(&PIND, DT) == 0)
  { 
    if(OCR1A < 32000)
      OCR1A += 22;
  }
  else if(GPIO_read(&PIND, DT) == 1)
  { 
    if(OCR1A > 16000)
      OCR1A -= 22;
  }    
}

ISR(ADC_vect)
{ 
  static uint16_t x_axis = 0;
  x_axis = ADC;

  if(x_axis < 400)
  { 
    if(OCR1B > 16000)
      OCR1B -= 22;
  }
  else if(x_axis > 625)
  { 
    if(OCR1B < 32000)
      OCR1B += 22;
  }
}