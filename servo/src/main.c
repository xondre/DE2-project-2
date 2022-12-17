/* Defines -----------------------------------------------------------*/

#define DT PD3    //
#define BUT PD7   //

#ifndef F_CPU
# define F_CPU 16000000 // CPU frequency in Hz required for delay funcs
#endif

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC            
#include <stdlib.h>         // C library. Needed for number conversions

/*  Global variables         */
uint8_t servo_select = 0;   //tracks which axis of joystick should be 
                            //read next, x axis controls servo on pin B1
                            //y axis controls servo on pin B2


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
  ICR1 = 39999;

  //fast PWM based on timer/counter 1, ICR1 as TOP
  TCCR1A |= (1<<WGM11); TCCR1A &= ~(1<<WGM10);
  TCCR1B |= (1<<WGM13) | (1<<WGM12);
  
  //prescaler for timer/counter 1 set to 8 = 010
  TCCR1B &= ~((1<<CS12) | (1<<CS10)); TCCR1B |= (1<<CS11);
  
  // OCR1x / ICR1 is the duty cycle
  // 3000/39999 at 50 Hz equals to pulse width of 1.5 ms, i.e. the 0° position
  OCR1A = 3000;
  OCR1B = 3000;

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
  
  //set timer2 overflow for 4 ms and enable timer2 overflow interrupt
  TIM2_overflow_4ms();
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

/************************************************************************
 * Function: TIMER2_OVF_vect
 * Purpose: Every 3 * 4ms, this routine triggers A-D conversion for either
            x axis or y axis of the joystick
 * Arguments: none
 * Returns: none
 ************************************************************************/
ISR(TIMER2_OVF_vect)
{
  //overflow counter
  static uint8_t counter = 0;
  
  //once the timer 2 overflows 3 times in a row
  if(counter < 4)
  { 
    counter++;
  }
  else
  { 
    //reset the overflow counter
    counter = 0;

    //perform A-D conversion for current joystick axis 
    if(servo_select == 0)
    { 
      // Select input channel ADC0 for x axis
      ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
      // Start ADC conversion
      ADCSRA |= (1<<ADSC);
      //set y axis as next A-D conversion
      servo_select = 1;
    }
    else
    { 
      // Select input channel ADC1 for y axis
      ADMUX |= (1<<MUX0);  ADMUX &= ~((1<<MUX1) | (1<<MUX2) | (1<<MUX3));
      // Start ADC conversion
      ADCSRA |= (1<<ADSC);
      //set x axis as next A-D conversion
      servo_select = 0;
    } 
  }
}


/************************************************************************
 * Function: ADC_vect
 * Purpose: Read current axis of the joystick and change corresponding
            pulse width if necessary
 * Arguments: none
 * Returns: none
 ************************************************************************/
ISR(ADC_vect)
{ 
  //values which track joystick position on each axis
  static uint16_t x_axis = 0;
  static uint16_t y_axis = 0;
  
  //depending on current servo selection
  if(servo_select == 0)
    x_axis = ADC;   //read x axis
  else
    y_axis = ADC;   //read y axis

  //each axis can have value from <0;1023>
  //when the joystick is tilted enough in one direction
  //axis value will be either under 300 or above 725
  if(x_axis < 300)
  { 
    //if pulse width on pin B1 is not too small 
    if(OCR1A > 700)
      OCR1A -= 22;    //decrement it
    //values for maximum and minimum of pulse width
    //and for de/incrementation were experimentally tuned
  }
  else if(x_axis > 725)
  { 
    //if pulse width on pin B2 is not too large
    if(OCR1A < 4800)
      OCR1A += 22;    //increment it
  }

  //same as above only for axis y (pin B2)
  if(y_axis < 300)
  { 
    if(OCR1B > 700)
      OCR1B -= 22;
  }
  else if(y_axis > 725)
  { 
    if(OCR1B < 4800)
      OCR1B += 22;
  }
}
