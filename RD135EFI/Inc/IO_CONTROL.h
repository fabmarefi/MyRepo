/*
 * SCHEDULLER.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_IO_CONTROL_H_
#define INC_IO_CONTROL_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "GEN_DEF.h"
#include "IO_CONTROL.h"

#define PWM_0                          0
#define PWM_100                     1001

#ifndef IO_CONTROL_c
extern TIM_HandleTypeDef htim4;
#endif

void Toggle_LED_Green(void);
void Set_Output_LED_Green(uint8_t	Value);
void Toggle_LED_Red(void);
void Set_Output_LED_Red(uint8_t	Value);
void Toggle_LED_Blue(void);
void Set_Output_LED_Blue(uint8_t Value);
void Toggle_LED_Yellow(void);
void Set_Output_LED_Yellow(uint8_t Value);
void Set_Ouput_Pump(uint8_t Value);
void TurnOffPump(void);
uint8_t Read_Output_Pump(void);
void Set_Ouput_Injector(uint8_t Value);
void Injector_CMD(uint16_t pwm);
void TurnOffInjector(void);
void Hardware_Init(void);

#endif /* INC_IO_CONTROL_H_ */

