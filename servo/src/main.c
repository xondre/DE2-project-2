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
  GPIO_mode_output(&DDRB, PB1);

  //fast PWM, ICR1 as TOP  
  TCCR1A |= (1<<WGM11); TCCR1A &= ~(1<<WGM10); TCCR1A |= (1<<COM1A1); TCCR1A &= ~(1<<COM1A0);
  TCCR1B |= ((1<<WGM13) | (1<<WGM12));
  //prescaler set to 256
  TCCR1B |= (1<<CS12); TCCR1B &= ~((1<<CS11) | (1<<CS10));
  //when prescaler=256 and ICR1=62450, PWM signal period is 20ms
  ICR1 = 62450;


    // Infinite loop
    while (1)
    {      
      OCR1A = 4683;
      _delay_ms(500);
      OCR1A = 3122;
      _delay_ms(500);
    }    

        return 0;   //this point is never reached
}

/* Interrupt service routines ----------------------------------------*/

