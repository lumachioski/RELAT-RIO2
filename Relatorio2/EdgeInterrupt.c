// RBG
// Runs on LM4F120 or TM4C123
// Request an interrupt on the falling edge of PF4 (when the user
// button is pressed) and increment a counter in the interrupt.  Note
// that button bouncing is not addressed.
// Maria Luiza Machioski Silvano
// November 18, 2021

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1, Program 9.4
   
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2, Program 5.6, Section 5.5

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// user button connected to PF4 (increment counter on falling edge)

#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))  // IRQ 28 to 31 Priority Register
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_PDR_R        (*((volatile unsigned long *)0x40025514))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define SYSCTL_RCGCGPIO_R       ( *( (volatile unsigned long * ) 0x400FE608 ) )
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
// user button connected to PF4 (increment counter on falling edge)

#include <stdint.h>

int duration;

void DisableInterrupts(void); // Disable interrupts

void EnableInterrupts(void);  // Enable interrupts

long StartCritical (void);    // previous I bit, disable interrupts

void EndCritical(long sr);    // restore I bit to previous value

void WaitForInterrupt(void);  // low power mode
//void Delaypadrao(void);
// global variable visible in Watch window of debugger

// increments at least once per button press

volatile uint32_t FallingEdges = 0;
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
	
void Delay(void);

void EdgeCounter_Init(void){                          

  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  FallingEdges = 0;             // (b) initialize counter

//GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)

  GPIO_PORTF_DIR_R = 0x1E;    // (c) make PF4 in (built-in button)
//GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
	GPIO_PORTF_AFSEL_R = 0x00;  //     disable alt funct on PF4
 //GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   

  GPIO_PORTF_DEN_R |= 0x1E;

	GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO

  GPIO_PORTF_AMSEL_R = 0;       //     diblob:https://web.whatsapp.com/6b46d50f-3fb9-4d78-938e-b9801968079esable analog functionality on PF

 // GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4

  GPIO_PORTF_PUR_R |= 0x11;

  GPIO_PORTF_IS_R &= 0x01;     // (d) PF4 is edge-sensitive

  GPIO_PORTF_IBE_R &= ~0x01;    //     PF4 is not both edges

  GPIO_PORTF_IEV_R &= ~0x01;    //     PF4 falling edge event

  GPIO_PORTF_ICR_R = 0x01;      // (e) clear flag4

  GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***

  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5

  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC

  EnableInterrupts();           // (i) Clears the I bit
}
void GPIOPortF_Handler(void){

  GPIO_PORTF_ICR_R = 0x01;      // acknowledge flag4

  FallingEdges = FallingEdges + 1;
}
//debug code
void delay( unsigned long duration)
{
	  duration = 2; 
    while ( ( duration -- )!= 0);
}

int main(void){
  EdgeCounter_Init();           // initialize GPIO Port F interrupt
  while(1){

   // WaitForInterrupt();
		if(FallingEdges==1)
		{
	GPIO_PORTF_DATA_R = 0x02;  
	delay(duration);
		}
				if(FallingEdges==2)
		{
	GPIO_PORTF_DATA_R = 0x04;  
	delay(duration);			
		}
				if(FallingEdges==3)
		{
	GPIO_PORTF_DATA_R = 0x08;  
	delay(duration);			
		}
				if(FallingEdges==4)
		{
	GPIO_PORTF_DATA_R = ~0x08;  
	delay(duration);			
		}
				if(FallingEdges==5)
		{
	GPIO_PORTF_DATA_R = ~0x04;  
	delay(duration);			
		}
				if(FallingEdges==6)
		{
	GPIO_PORTF_DATA_R = ~0x02;  
	delay(duration);
		}
				if(FallingEdges==7)
		{
	GPIO_PORTF_DATA_R = ~0x00;  
	delay(duration);
		}
		if(FallingEdges==8)
{
	FallingEdges=1;
}	

	}
	}
