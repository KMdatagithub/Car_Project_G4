/*
 * timers.h
 *
 *  Created on: Oct 22, 2019
 *      Author: Sprints
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "registers.h"
#include "gpio.h"

/*
 * User Configuration Macros
 */

#define T0_PWM_GPIO	GPIOD
#define T0_PWM_BIT	BIT0

#define T1_PWM_GPIO	GPIOD
#define T1_PWM_BIT	BIT1

#define T2_PWM_GPIO GPIOD
#define T2_PWM_BIT	BIT2

#define TOV0   ((TIFR>>0)&0x01)

//---------------------------
// for timer 0

typedef enum En_frequancy_t{
	F_8000,F_2000,F_1000,F_500,F_250,F_60
}En_frequancy_t;

typedef enum En_timer0Mode_t{
	T0_NORMAL_MODE=0,T0_COMP_MODE=0x08
}En_timer0Mode_t;

typedef enum En_timer0OC_t{
	T0_OC0_DIS=0,T0_OC0_TOGGLE=0x10,T0_OC0_CLEAR=0x20,T0_OC0_SET=0x30
}En_timer0OC_t;

typedef enum En_timer0perscaler_t{
	T0_NO_CLOCK=0,T0_PRESCALER_NO=0x01,T0_PRESCALER_8=0x02,T0_PRESCALER_64=0x03,T0_PRESCALER_256=0x04,T0_PRESCALER_1024=0x05
}En_timer0perscaler_t;

typedef enum En_timer0Interrupt_t{
	T0_POLLING=0,T0_INTERRUPT_NORMAL=0x01,T0_INTERRUPT_CMP=0x02 , T0_ALL_INTERRUPT=0x03
}En_timer0Interrupt_t;

//---------------------------
// for timer 1

typedef enum En_timer1Mode_t{
	T1_NORMAL_MODE=0x0000,T1_COMP_MODE_OCR1A_TOP=0x0008, T1_COMP_MODE_ICR1_TOP = 0x0018

}En_timer1Mode_t;
typedef enum En_timer1OC_t{
	T1_OC1_DIS=0,T1_OC1A_TOGGLE=0x4000,T1_OC1B_TOGGLE=0x1000,T1_OC1A_CLEAR=0x8000,T1_OC1B_CLEAR=0x2000,T1_OC1A_SET=0xC000,T1_OC1B_SET=0x3000
}En_timer1OC_t;

typedef enum En_timer1perscaler_t{
	T1_NO_CLOCK=0x0000,T1_PRESCALER_NO=0x0001,T1_PRESCALER_8=0x0002,T1_PRESCALER_64=0x0003,T1_PRESCALER_256=0x0004,T1_PRESCALER_1024=0x0005
}En_timer1perscaler_t;

typedef enum En_timer1Interrupt_t{
	T1_POLLING=0,T1_INTERRUPT_NORMAL=0x04,T0_INTERRUPT_CMP_1B=0x08, T1_INTERRUPT_CMP_1A=0x10, T1_INTERRUPT_ICAPTURE = 0x20
}En_timer1Interrupt_t;

//---------------------------------
// for timer 2

typedef enum En_timer2Mode_t{
	T2_NORMAL_MODE=0,T2_COMP_MODE=0x08
}En_timer2Mode_t;

typedef enum En_timer2OC_t{
	T2_OC2_DIS=0,T2_OC2_TOGGLE=0x10,T2_OC2_CLEAR=0x20,T2_OC2_SET=0x30
}En_timer2OC_t;

typedef enum En_timer2perscaler_t{
	T2_NO_CLOCK=0,T2_PRESCALER_NO=0x01,T2_PRESCALER_8=0x02, T2_PRESCALER_32=0x03, T2_PRESCALER_64=0x04, T2_PRESCALER_128=0x05, T2_PRESCALER_256 = 0x06, T2_PRESCALER_1024=0x07
}En_timer2perscaler_t;

typedef enum En_timer2Interrupt_t{
	T2_POLLING=0,T2_INTERRUPT_NORMAL=0x40,T2_INTERRUPT_CMP=0x80,T2_ALL_INTERRUPT=(0X40|0X80)
}En_timer2Interrupt_t;

/*===========================Timer0 Control===============================*/
/*
 * Description: It Initializes Timer0 Module
 * @param en_mode:           Set Timer0 Mode OF Operation
 * @param en_OC0:            Set Output Compare Mode (enabled, disabled, etc)
 * @param en_prescal:        Sets The Timer0 Prescaler Value (8, 64, 256, 1024, NO)
 * @param u8_initialValue:   Sets The Timer0 Initial TCNT0 Ticks Count
 * @param u8_outputCompare:  Sets The Output Compare Register Value OCR0
 * @param en_interruptMask:  Sets The Interrupt Mode For TMR0 (polling, overflow, or compare match)
 */
void timer0Init(En_timer0Mode_t en_mode,En_timer0OC_t en_OC0,En_timer0perscaler_t en_prescal, uint8_t u8_initialValue, uint8_t u8_outputCompare, En_timer0Interrupt_t en_interruptMask);
/*
 * Description: It Sets The Start Value For Timer0 Ticks
 * @param u8_value: Sets The TMR0 Initial Value
 */
void timer0Set(uint8_t u8_value);
/*
 * Description: It Returns The Number OF Ticks In TCNT0
 * @return: The Current Value OF Timer0 Ticks
 */
uint8_t timer0Read(void);
/*
 * Description: It Sets The TMR0 Prescaler, So It Starts Counting!
 */
void timer0Start(void);
/*
 * Description: It Clears The TMR0 Prescaler, So It Stops Counting!
 */
void timer0Stop(void);
/*
 * Description: It Halts The CPU For A Specified Amount OF Time (in msec)
 * @param u16_delay_in_ms: Number OF Milli_seconds For The Delay
 */
void timer0DelayMs(uint16_t u16_delay_in_ms);
/*
 * Description: It Halts The CPU For A Specified Amount OF Time (in usec)
 * @param u32_delay_in_us: Number OF Micro_seconds For The Delay
 */
void timer0DelayUs(uint32_t u32_delay_in_us);
/*
 * Description: It Generates A Soft PWM Signal
 * @param u8_dutyCycle: The User Desired DutyCycle [0 - 255]
 * @param u8_frequency: The User Desired Frequency [KHz]
 */
void timer0SwPWM(uint8_t u8_dutyCycle,uint8_t u8_frequency);


//--------------------------------------------------------------------------
/*===========================Timer2 Control===============================*/
/*
 * Description: It Initializes Timer2 Module
 * @param en_mode:           Set Timer2 Mode OF Operation
 * @param en_OC:             Set Output Compare Mode (enabled, disabled, etc)
 * @param en_prescal:        Sets The Timer2 Prescaler Value (8, 64, 256, 1024, NO)
 * @param u8_initialValue:   Sets The Timer2 Initial TCNT2 Ticks Count
 * @param u8_outputCompare:  Sets The Output Compare Register Value OCR2
 * @param u8_assynchronous: 
 * @param en_interruptMask:  Sets The Interrupt Mode For TMR2 (polling, overflow, or compare match)
 */
void timer2Init(En_timer2Mode_t en_mode,En_timer2OC_t en_OC,En_timer2perscaler_t en_prescal, uint8_t u8_initialValue, uint8_t u8_outputCompare, uint8_t u8_assynchronous, En_timer2Interrupt_t en_interruptMask);
/*
 * Description: It Sets The Start Value For Timer2 Ticks
 * @param u8_value: Sets The TMR2 Initial Value
 */
void timer2Set(uint8_t u8_a_value);
/*
 * Description: It Returns The Number OF Ticks In TCNT2
 * @return: The Current Value OF Timer2 Ticks
 */
uint8_t timer2Read(void);
/*
 * Description: It Sets The TMR2 Prescaler, So It Starts Counting!
 */
void timer2Start(void);
/*
 * Description: It Clears The TMR2 Prescaler, So It Stops Counting!
 */
void timer2Stop(void);
/*
 * Description: It Halts The CPU For A Specified Amount OF Time (in msec)
 * @param u16_delay_in_ms: Number OF Milli_seconds For The Delay
 */
void timer2DelayMs(uint16_t u16_delay_in_ms);
/*
 * Description: It Halts The CPU For A Specified Amount OF Time (in usec)
 * @param u32_delay_in_us: Number OF Micro_seconds For The Delay
 */
void timer2DelayUs(uint32_t u16_delay_in_us);
/*
 * Description: It Generates A Soft PWM Signal
 * @param u8_dutyCycle: The User Desired DutyCycle [0->255]
 * @param u8_frequency: The User Desired Frequency [KHz]
 */
void timer2SwPWM(uint8_t u8_dutyCycle,uint8_t u8_frequency);

#endif /* TIMERS_H_ */