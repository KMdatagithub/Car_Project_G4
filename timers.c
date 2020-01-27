/*
 * timers.c
 *
 * Created: 1/20/2020 11:06:28 PM
 *  Author: ahmed
 */ 
#include "timers.h"

En_timer0perscaler_t en_gprescal ;
uint8_t u8_gTCNT0PreloadValue;

volatile static uint8_t f16_dutyCycle = 0;

void timer0Init(En_timer0Mode_t en_mode,En_timer0OC_t en_OC0,En_timer0perscaler_t en_prescal, uint8_t u8_initialValue, uint8_t u8_outputCompare, En_timer0Interrupt_t en_interruptMask)
{
	en_gprescal = en_prescal;
	
	switch (en_mode)
	{
		case T0_NORMAL_MODE:
		TCCR0 |= T0_NORMAL_MODE;
		break;
		
		case T0_COMP_MODE:
		TCCR0 |= T0_COMP_MODE;
		break;
	}
	
	switch (en_OC0)
	{
		case T0_OC0_TOGGLE:
		TCCR0 |= T0_OC0_TOGGLE;
		break;
		
		case T0_OC0_SET:
		TCCR0 |= T0_OC0_SET;
		break;
		
		case T0_OC0_CLEAR:
		TCCR0 |= T0_OC0_CLEAR;
		break;
		
		case T0_OC0_DIS:
		TCCR0 |= T0_OC0_DIS;
		
	}
	
	switch (en_prescal)
	{
		case T0_NO_CLOCK:
		TCCR0 |= T0_NO_CLOCK;
		break;
		
		case T0_PRESCALER_NO:
		TCCR0 |= T0_PRESCALER_NO;
		break;
		
		case T0_PRESCALER_8:
		TCCR0 |= T0_PRESCALER_8;
		break;
		
		case T0_PRESCALER_64:
		TCCR0 |= T0_PRESCALER_64;
		break;
		
		case T0_PRESCALER_256:
		TCCR0 |= T0_PRESCALER_256;
		break;
		
		case T0_PRESCALER_1024:
		TCCR0 |= T0_PRESCALER_1024;
		break;
	}
	
	switch (en_interruptMask)
	{
		case T0_POLLING:
		TIMSK |= T0_POLLING;
		break;
		
		case T0_INTERRUPT_NORMAL:
		TIMSK |= T0_INTERRUPT_NORMAL;
		break;
		
		case T0_INTERRUPT_CMP:
		TIMSK |= T0_INTERRUPT_CMP;
		break;
		
		case T0_ALL_INTERRUPT:
		TIMSK |= T0_ALL_INTERRUPT;
		break;
	}
	
	TCNT0 = u8_initialValue;
	OCR0  = u8_outputCompare;
}

void timer0Set(uint8_t u8_value)
{
	TCNT0 = u8_value;
}

uint8_t timer0Read(void)
{
	return TCNT0;
}

void timer0Stop(void)
{
	TCCR0>>=3;
	TCCR0<<=3;
}

void timer0Start(void)
{
	TCCR0|=en_gprescal;
}

void timer0DelayMs(uint16_t u16_delay_in_ms)
{
	timer0Init(T0_NORMAL_MODE,T0_OC0_DIS,T0_PRESCALER_8,0,0,T0_POLLING);
	uint32_t u32_OVFCounter;
	uint8_t  u8_ISRCount=1;
	u8_gTCNT0PreloadValue=48;
	TCNT0 = u8_gTCNT0PreloadValue;
	
	timer0Start();
	
	for (u32_OVFCounter=0;u32_OVFCounter<(8*u16_delay_in_ms);u32_OVFCounter++)
	{
		
		while (!TOV0);
		u8_ISRCount++;
		if(u8_ISRCount == 8)
		{
			u8_gTCNT0PreloadValue=48;
			u8_ISRCount=1;
		}
		TIFR |= (TIFR<<0);	
	}
	
	timer0Stop();
}

void timer0DelayUs(uint32_t u32_delay_in_us)
{
	uint32_t u32_OVFCounter;
	u8_gTCNT0PreloadValue=254;
	TCNT0 = u8_gTCNT0PreloadValue;
	
	timer0Start();
	for (u32_OVFCounter=0;u32_OVFCounter<(u32_delay_in_us);u32_OVFCounter++)
	{
		while (!TOV0);
		TIFR |= (TIFR<<0);
	}

	timer0Stop();
}

void timer2Init(En_timer2Mode_t en_mode,En_timer2OC_t en_OC,En_timer2perscaler_t en_prescal, uint8_t u8_initialValue, uint8_t u8_outputCompare, uint8_t u8_assynchronous, En_timer2Interrupt_t en_interruptMask)
{
	en_gprescal = en_prescal;
	
	switch (en_mode)
	{
		case T2_NORMAL_MODE:
		TCCR2 |= T2_NORMAL_MODE;
		break;
		
		case T2_COMP_MODE:
		TCCR2 |= T2_COMP_MODE;
		break;
	}
	
	switch (en_OC)
	{
		case T2_OC2_TOGGLE:
		TCCR2 |= T2_OC2_TOGGLE;
		break;
		
		case T2_OC2_SET:
		TCCR2 |= T2_OC2_SET;
		break;
		
		case T2_OC2_CLEAR:
		TCCR2 |= T2_OC2_CLEAR;
		break;
		
		case T2_OC2_DIS:
		TCCR2 |= T2_OC2_DIS;
		
	}
	
	switch (en_prescal)
	{
		case T2_NO_CLOCK:
		TCCR2 |= T2_NO_CLOCK;
		break;
		
		case T2_PRESCALER_NO:
		TCCR2 |= T2_PRESCALER_NO;
		break;
		
		case T2_PRESCALER_8:
		TCCR2 |= T2_PRESCALER_8;
		break;
		
		case T2_PRESCALER_32:
		TCCR2 |= T2_PRESCALER_32;
		break;
		
		case T2_PRESCALER_64:
		TCCR2 |= T2_PRESCALER_64;
		break;
		
		case T2_PRESCALER_128:
		TCCR2 |= T2_PRESCALER_128;
		break;
		
		case T2_PRESCALER_256:
		TCCR2 |= T2_PRESCALER_256;
		break;
		
		case T2_PRESCALER_1024:
		TCCR2 |= T2_PRESCALER_1024;
		break;
		
	}
	
	switch (en_interruptMask)
	{
		case T2_POLLING:
		TIMSK |= T2_POLLING;
		break;
		
		case T2_INTERRUPT_NORMAL:
		TIMSK |= T2_INTERRUPT_NORMAL;
		break;
		
		case T2_INTERRUPT_CMP:
		TIMSK |= T2_INTERRUPT_CMP;
		break;
		
		case T2_ALL_INTERRUPT:
		TIMSK |= T2_ALL_INTERRUPT;
		break;
	}
	
	TCNT2 = u8_initialValue;
	OCR2  = u8_outputCompare;
}

void timer2SwPWM(uint8_t u8_dutyCycle,uint8_t u8_frequency)
{
	//f16_dutyCycle = (uint8_t) ((u8_dutyCycle/100.0)*(255.0)) ;
	//OCR2 = f16_dutyCycle*255;
	//OCR2 = f16_dutyCycle;
	//f16_dutyCycle = u8_dutyCycle;
	OCR2 = u8_dutyCycle;
	switch(u8_frequency)
	{
		case F_8000:
		timer2Init(T2_NORMAL_MODE,T2_OC2_DIS,T2_PRESCALER_8, 0, OCR2,0, T2_ALL_INTERRUPT);
		break;
		
		case F_2000:
		timer2Init(T2_NORMAL_MODE,T2_OC2_DIS,T2_PRESCALER_32, 0, OCR2,0, T2_ALL_INTERRUPT);
		break;
		
		case F_1000:
		timer2Init(T2_NORMAL_MODE,T2_OC2_DIS,T2_PRESCALER_64, 0, OCR2,0, T2_ALL_INTERRUPT);
		break;
		
		case F_500:
		timer2Init(T2_NORMAL_MODE,T2_OC2_DIS,T2_PRESCALER_128, 0, OCR2,0, T2_ALL_INTERRUPT);
		break;
		
		case F_250:
		timer2Init(T2_NORMAL_MODE,T2_OC2_DIS,T2_PRESCALER_256, 0, OCR2,0, T2_ALL_INTERRUPT);
		break;
		
		case F_60:
		timer2Init(T2_NORMAL_MODE,T2_OC2_DIS,T2_PRESCALER_1024, 0, OCR2,0, T2_ALL_INTERRUPT);
		break;
	}
}
