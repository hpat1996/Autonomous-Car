// PB0: Bluetooth
#define GPIO_PB0_U1RX           0x00010001

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>
#include "tm4c123gh6pm.h"

bool INTERRUPT = false;
int duty = 80;
int aDuty = 80;

int change = 0;

bool autopilot = false;
bool up = false;
bool left = false;
bool right = false;
bool stop = true;

void LEDInit()
{
  // Port F
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;

  GPIO_PORTF_DIR_R |= 0x0E;      // Set LEDS as output
  GPIO_PORTF_DEN_R |= 0x0E;      // Set all as enabled
  GPIO_PORTF_DATA_R = 0x00;      // Turn off all LED
}

void UARTInit()
{
  
  // Enable UART1 and GPIOB
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  // Enable PB0 to receive data and set it as a UART type.
  GPIOPinConfigure(GPIO_PB0_U1RX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);
  
  // Set the clock and configure the UART using BAUD rate and clock frequency
  UARTClockSourceSet(UART1_BASE,UART_CLOCK_SYSTEM);
  UARTConfigSetExpClk(UART1_BASE,SysCtlClockGet(),9600,(UART_CONFIG_WLEN_8|UART_CONFIG_PAR_NONE|UART_CONFIG_STOP_ONE));
  
  // Enable FIFO
  UARTFIFOEnable(UART1_BASE);
  
  if (INTERRUPT) {
    // If we want UART interrupt handler.
    IntEnable(INT_UART1);
    //IntPrioritySet(INT_UART1, 3);
    UARTIntEnable(UART1_BASE, UART_IM_RXIM);
  }
}

// Turn on the controls based on the keys pressed in the app.
void Control() {
  if (stop) {
    // If stop is active.
    // Turn on the red LED (for debugging purposes)
    // Call the stop function from wheel_control
    GPIO_PORTF_DATA_R = 0x02;
    Stop();
  } else if (left) {
    // If left is active.
    // Turn on the red + green LED (for debugging purposes)
    // Call the left function from wheel_control
    GPIO_PORTF_DATA_R = 0x0C;
    Left(duty);
  } else if (right) {
    // If right is active.
    // Turn on the red + blue LED (for debugging purposes)
    // Call the right function from wheel_control
    GPIO_PORTF_DATA_R = 0x06;
    Right(duty);
  } else if (up) {
    // If up is active.
    // Turn on the blue LED (for debugging purposes)
    // Call the up function from wheel_control
    GPIO_PORTF_DATA_R = 0x04;
    Up(duty);
  } else {
    // By defualt LED is off.
    // Stay in stop mode.
    // Call the stop function from wheel_control
    GPIO_PORTF_DATA_R = 0x00;
    Stop(duty);
  }
}

// Get the config of controls from app.
void direction(char data) {
  if (data == 'a') {
    // The autopilot button was clicked.
    autopilot = !autopilot;
    stop = false;
    up = false;
    left = false;
    right = false;
  } else if (data == 's') {
    // The stop button was clicked.
    autopilot = false;
    stop = !stop;
    up = false;
    left = false;
    right = false;
  } else if (data == 'l') {
    //The left button was clicked.
    left = !left;
    right = false;
    stop = false;
  } else if (data == 'r') {
    // The right button was clicked.
    left = false;
    right = !right;
    stop = false;
  } else if (data == 'u') {
    // The up button was clicked.
    up = !up;
    stop = false;
  } else {
    up = false;
    autopilot = false;
    left = false;
    right = false;
    stop = true;
  }
}

// The UART handler.
// Disabled by default.
void UART_Handler() {
  char data = UARTCharGet(UART1_BASE);
  direction(data);
  UARTIntClear(UART1_BASE, UART_IM_RXIM);
}

int main(void) {
  // Initialize the PortF LED (for debugging purposes)
  LEDInit();
  // Initialize UART for bluetooth module.
  UARTInit();
  // Initialize PortD for infrared sensor.
  PortD_Init();
  // Initialize PortA for wheel controls and motors.
  // by default, the car starts from stop state.
  Motor_Init12(1);
  // Turn on the red LED (for debugging purposes.
  GPIO_PORTF_DATA_R = 0x02;
  
  while(1) {
    if (!INTERRUPT) {
      // Interrupt is disabled by defult; using polling.
      // Wait for instruction from bluetooth app.
      if (UARTCharsAvail(UART1_BASE)) {
        char data = UARTCharGet(UART1_BASE);
        // After new data is received, set the new directions; change the state.
        direction(data);
        Control();
      }

      if (autopilot) {
        // If autopilot is active.
        // Turn on green LED (for debugging purposes)
        // Call autopilot from infrared_control
        GPIO_PORTF_DATA_R = 0x08;
        for (int i = 0 ; i < 1000; i++) {}
        change = Autopilot(duty, change);
      }

    }
  }
}