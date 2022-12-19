# Project 2 - servo motor control

This project wants to show an example of servo motor control using PWM signal produced by Arduino UNO. Two servo motors are controlled using an analog joystick and can produce movement in a mechanical structure which is supposed to represent part of a robotic arm.  

### Team members

* Adam Ondrejka
* Dominik Vaško

## Hardware description

Project uses 2 SG90 servo motors, Arduino UNO and analog joystick. Servos are powered from Arduino's +5V rail as is the joystick. For the servo control, pins B1 and B2 (9 and 10) are used, because they can output PWM signal based on timer/counter 1. Outputs from the 2 axis of analog joystick are connected to pins A0 and A1 because these pins support A-D conversion. The servo motors were mounted to a metal structure made from Merkur construction set with a help of zip ties, as can be seen in the video located at the end of this text. 
![schematic](schematic.png)

## Software description

Each servo requires PWM input, the pulse width determines servo's position. Servo motor SG90 is capable of rotating from -90° to 90° (pulse width from 1 ms to 2 ms) with respect to 0° (1.5 ms)[[1]](https://robojax.com/learn/arduino/robojax-servo-sg90_datasheet.pdf). Datasheet of SG90 also mentions PWM frequency of 50 Hz (period of 20 ms), this frequency was therefore used in the project. It should be noted that the pulse width range from 1 ms to 2 ms in practice did not result in -90° to 90° range of movement, so it had to be modified experimentally.

[Main source file](https://github.com/xondre/DE2-project-2/blob/main/servo/src/main.c) - contains the source code of whole program 

[GPIO library](https://github.com/xondre/DE2-project-2/tree/main/servo/lib/gpio) - this library was used for easy set up of output pins

[timer header file](https://github.com/xondre/DE2-project-2/blob/main/servo/include/timer.h) - this header file includes macros, which simplify control of timers

### main()
The main() function is called first in the program, its first part are inital settings for output pins, PWM, A-D converter and timer/counter 2 interrupt[[2]](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf). PWM frequency of 50 Hz is achieved by custom TOP value in register ICR1 which determines how high the timer/counter 1 will count with a given prescaler. The pulse width can then be controlled by changing the value in register OCR1A (pin B1) or OCR1B (pin B2) from 0 to TOP[[2]](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf). Last part of main() is the empty loop which represents the runtime after the start, it is empty because everything else is handled using TIMER2_OVF_vect and ADC_vect.

![main](main.png)

### TIMER2_OVF_vect
TIMER2_OVF_vect is used to trigger ADC_vect approximately every 12 ms. Since analog values from pins A0 and A1 are needed, the ADC channel is changed every cycle (12 ms) between ADC0 and ADC1.  

![timer2](timer2.png)

### ADC_vect
Joystick has 2 axis, x and y, analog value on each axis then represents joystick's orientation. ADC_vect reads analog value from one axis at the time and then checks if the value lies close to the upper or lower end of the axis, then it checks if OCR1x register value still lies in the allowed range, if so, the OCR1x is incremented or decremented, this changes the pulse width and therefore position of the corresponding servo motor.

![ADC](ADC.png)

## Video

[Short video of our practical implementation.](https://www.youtube.com/watch?v=cIwn9iO3daw)

## References

1. [SG90 datasheet](https://robojax.com/learn/arduino/robojax-servo-sg90_datasheet.pdf)
2. [ATmega328P datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf) - microcontroller inside Arduino UNO
3. [program used to create the schematic](https://www.simulide.com/p/home.html)
4. [app used to create the flowcharts](https://app.diagrams.net/)
