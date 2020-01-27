/*
 * dcMotor.h
 *
 * Created: 1/26/2020 10:28:41 AM
 *  Author: Khaled Magdy
 */ 


#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "gpio.h"
#include "timers.h"

//--------------------[ Motor Control Pin Definitions ]---------------------
//--------------------------------------------------------------------------
#define M1EN_GPIO	(GPIOD)
#define M1EN_BIT	(BIT4)

#define M2EN_GPIO	(GPIOD)
#define M2EN_BIT	(BIT5)

#define M1D1_GPIO	(GPIOD)
#define M1D1_BIT	(BIT2)

#define M1D2_GPIO	(GPIOD)
#define M1D2_BIT	(BIT3)

#define M2D1_GPIO	(GPIOD)
#define M2D1_BIT	(BIT6)

#define M2D2_GPIO	(GPIOD)
#define M2D2_BIT	(BIT7)
//---------------------------------------------------------------------
//----------------------[ DC Motor 1 Control ]-------------------------
/**
 * Description:  Enables The EN1 Line For Motor 1
 * @param void
 */
void dcMotor1Enable(void);
/**
 * Description: Disables The EN1 Line For Motor1 & Clears The DutyCycle
 * @param void
 */
void dcMotor1Disable(void);
/**
 * Description:  Sets The DutyCycle For Motor1 & The Rotation Direction
 * @param u8_dutyCycle Takes The DutyCycle Ranging From (0-255)
 * @param u8_direction Takes The Rotation Direction (0 or 1) For CW / CCW
 */
void dcMotor1SetDCD(uint8_t u8_dutyCycle, uint8_t u8_direction);
//---------------------------------------------------------------------
//----------------------[ DC Motor 2 Control ]-------------------------
/**
 * Description: Enables The EN2 Line For Motor 2
 * @param void
 */
void dcMotor2Enable(void);
/**
 * Description: Disables The EN2 Line For Motor2 & Clears The DutyCycle
 * @param void
 */
void dcMotor2Disable(void);
/**
 * Description:  Sets The DutyCycle For Motor2 & The Rotation Direction
 * @param u8_dutyCycle Takes The DutyCycle Ranging From (0-255)
 * @param u8_direction Takes The Rotation Direction (0 or 1) For CW / CCW
 */
void dcMotor2SetDCD(uint8_t u8_dutyCycle, uint8_t u8_direction);
//---------------------------------------------------------------------
//----------------------[ Robotic Car Control ]------------------------
/**
 * Description:  Drives The Robotic Car Forward
 * @param u8_dutyCycle Sets The DutyCycle For The Motors
 */
void MoveForward(uint8_t u8_dutyCycle);
/**
 * Description:  Drives The Robotic Car Backwards
 * @param u8_dutyCycle Set The DutyCycle For The Motors
 */
void MoveBackward(uint8_t u8_dutyCycle);
/**
 * Description:  Rotates The Car ClockWise
 * @param u8_dutyCycle Set The DutyCycle For The Motors Ranging From 0 to 255
 */
void CWrotate(uint8_t u8_dutyCycle);
/**
 * Description:  Rotates The Car Counter Clock Wise
 * @param u8_dutyCycle Set The DutyCycle For The Motors Ranging From 0 to 255
 */
void CCWrotate(uint8_t u8_dutyCycle);
/**
 * Description:  Stops The Car!
 * @param void
 */
void carSTOP(void);

#endif /* DCMOTOR_H_ */