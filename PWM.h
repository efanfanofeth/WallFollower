// PWM.h
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// Modified by Min He, September 7, 2021
#include <stdint.h>

// period is 16-bit number of PWM clock cycles in one period 
// Output on PB6/M0PWM0
void PWM0A_Init(uint16_t period);

// change duty cycle of PB6
// duty is number of PWM clock cycles output is high  
void PWM0A_Duty(uint16_t duty);

// period is 16-bit number of PWM clock cycles in one period 
// Output on PB7/M0PWM1
void PWM0B_Init(uint16_t period);

// change duty cycle of PB7
// duty is number of PWM clock cycles output is high  
void PWM0B_Duty(uint16_t duty);

