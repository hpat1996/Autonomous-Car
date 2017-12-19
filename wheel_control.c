// PA4 : Left
// PA5 : Right

#include "tm4c123gh6pm.h"
#include <stdint.h>

void LorR(int count);

unsigned long highL, lowL;
unsigned long highR, lowR;
unsigned long highLR, lowLR;

int count;

int PA4 = 0x10;
int PA5 = 0x20;

void configPA(int PA) {
  GPIO_PORTA_AMSEL_R &= ~PA;   //PA
  GPIO_PORTA_PCTL_R &= ~0x00F00000;
  GPIO_PORTA_DIR_R |= PA;     // make PA out
  GPIO_PORTA_DR8R_R |= PA;    // enable 8 mA drive on PA
  GPIO_PORTA_AFSEL_R &= ~PA;  // disable alt funct on PA
  GPIO_PORTA_DEN_R |= PA;     // enable digital I/O on PA
  GPIO_PORTA_DATA_R &= ~PA;   // make PA low
}

void sysTickConfig(unsigned long high) {
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = high-1;    // reload value for 500us
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}

void Motor_Init1(int duty){
  SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A

  highL = 10000*(100-duty)/100;
  lowL = 10000-highL;

  configPA(PA5);
  sysTickConfig(highL);
  count = 1; 
}

void Motor_Init2(int duty){
  SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A

  highR = 10000*(100-duty)/100;
  lowR = 10000-highR;

  configPA(PA4);
  sysTickConfig(highR);         // reload value for 500us
  count = 2; 
}

void Motor_Init12(int duty){
  SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A

  highLR = 10000*(100-duty)/100;
  lowLR = 10000-highLR;

  configPA(PA4);
  configPA(PA5);

  sysTickConfig(highLR);       // reload value for 500us

  count = 3;
}

void SysTick_Handler(void){
  LorR(count);
}

void phasePA(int PA, unsigned long high, unsigned long low) {
  if(GPIO_PORTA_DATA_R&PA){   // toggle PA
    GPIO_PORTA_DATA_R &= ~PA; // make PA low
    NVIC_ST_RELOAD_R = high-1;    // reload value for low phase
  } else{
    GPIO_PORTA_DATA_R |= PA;      // make PA high
    NVIC_ST_RELOAD_R = low-1;     // reload value for high phase
  }
}

void LorR(int count) {
  if (count == 1) {
    phasePA(PA5, highL, lowL);
  } else if (count == 2) {
    phasePA(PA4, highR, lowR);
  } else {
    phasePA(PA4, highLR, lowLR);
    phasePA(PA5, highLR, lowLR);
  }
}

void Left(int duty) {
  Motor_Init1(1);
  for (int i = 0 ; i < 50000; i++) {}
  Motor_Init2(duty);
}

void Right(int duty) {
  Motor_Init2(1);
  for (int i = 0 ; i < 50000; i++) {}
  Motor_Init1(duty);
}

void Up(int duty) {
  Motor_Init12(duty);
}

void Stop(void) {
  Motor_Init12(1);
}