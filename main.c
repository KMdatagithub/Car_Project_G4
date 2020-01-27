/*
 * Created: 1/26/2020 11:34:52 AM
 * Authors: Group4 (Jevara, Khaldoon, Ramadan, Boda)
 */ 
#include "std_types.h"
#include "registers.h"
#include "gpio.h"
#include "interrupt.h"
#include "timers.h"
#include "dcMotor.h"

int main(void)
{
	uint8_t u8_dutyCycle=0;
	// Enable Interrupts & DC Motors 1, 2
	SREG |=0b10000000;
	dcMotor1Enable();
	dcMotor2Enable();
	while (1)
	{
		//----------[ Accelerate Forward From 0% to 100% ]----------
		for (u8_dutyCycle=0;u8_dutyCycle<255;u8_dutyCycle++)
		{
			MoveForward(u8_dutyCycle);
			timer0DelayMs(20);
		}
		//----------[ Decelerate Forward From 100% to 0% ]----------
		for (u8_dutyCycle=255;u8_dutyCycle>0;u8_dutyCycle--)
		{
			MoveForward(u8_dutyCycle);
			timer0DelayMs(20);
		}
		//----------[ Turn ClockWise 90 Degrees ]----------
		CCWrotate(100);
	    timer0DelayMs(200);
		//----------[ Stop The Car ]----------
		dcMotor1Disable();
		dcMotor2Disable();
		while(1);
	}
}

//------------[ Timer2 Overflow ISR ]-------------
InterruptServiceRoutine(TIMER2_OVF_vect)
{
	gpioPinWrite(M1EN_GPIO, M1EN_BIT,HIGH);
	gpioPinWrite(M2EN_GPIO, M2EN_BIT,HIGH);
}
//------------[ Timer2 COMP Match ISR ]-------------
InterruptServiceRoutine(TIMER2_COMP_vect)
{
	gpioPinWrite(M1EN_GPIO, M1EN_BIT,LOW);
	gpioPinWrite(M2EN_GPIO, M2EN_BIT,LOW);
}