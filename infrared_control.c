// Program that detects black tape using two sensors on PD6 and PD7

#include "tm4c123gh6pm.h"
#include <stdint.h>

#define PD6 (*((volatile unsigned long *)0x40007100)) // left sensor
#define PD7 (*((volatile unsigned long *)0x40007200)) // right sensor 

int PortD_Init () {
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD; // linking clock to Port D Gating Control
  GPIO_PORTD_LOCK_R |=  0x4C4F434B;  // unlock port D
  GPIO_PORTD_CR_R = 0xF0;           // allow changes to PD7-4
  // only PD7 needs to be unlocked, other bits can't be locked
  GPIO_PORTD_AMSEL_R &= ~0xC0;       // disable analog on PD6 and PD7
  GPIO_PORTD_PCTL_R &= ~0xFF000000;  // PCTL GPIO on PD6 and PD7
  GPIO_PORTD_DIR_R &= ~0xC0;         // direction PD6 and PD7 input
  GPIO_PORTD_AFSEL_R &= ~0xC0;       // PD6 and PD7 regular port function
  GPIO_PORTD_DEN_R |= 0xC0;          // enable PD6 and PD7 digital port
  return 0;
}

int Autopilot(int duty, int change) {
  if (PD6) { // left sensor detects tape
    GPIO_PORTF_DATA_R = 0x02; // turn left
    Left(duty);
    return 1;
  } 
  if (PD7) { // right sensor detects tape
    GPIO_PORTF_DATA_R = 0x02; // turn right
    Right(duty);
    return 1;
  }
  if (change) {
    GPIO_PORTF_DATA_R = 0x00; // go up
    Up(duty);
    return 0;
  }
}
