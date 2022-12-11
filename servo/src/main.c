/* Defines -----------------------------------------------------------*/

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
  OCR1A = 16000;
  OCR1B = 32000;

  //set timer2 overflow for 2 ms and enable interrupt
  TIM2_overflow_2ms();
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
  static uint16_t counter = 16000;
  static int8_t step = 16;

  OCR1A = counter += step;

  if(counter == 32000)
  { 
    step = -16;
  }
  else if(counter == 16000)
  { 
    step = 16;
  }

}