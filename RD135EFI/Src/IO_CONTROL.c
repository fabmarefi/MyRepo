/*
 * IO_CONTROL.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#define IO_CONTROL_c

#include "IO_CONTROL.h"

//I needed to declarate here so that function could use it...
TIM_HandleTypeDef htim4;

//Led Green (Bluepill)
void Toggle_LED_Green(void)
{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}

void Set_Output_LED_Green(uint8_t	Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		}
}

//Led Red
void Toggle_LED_Red(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
}

void Set_Output_LED_Red(uint8_t	Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		}
}

//Led Blue
void Toggle_LED_Blue(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
}

void Set_Output_LED_Blue(uint8_t Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		}
}

//Led Yellow
void Toggle_LED_Yellow(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
}

void Set_Output_LED_Yellow(uint8_t Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		}
}

//Pump
void Set_Ouput_Pump(uint8_t Value)
{
    if (Value==ON)
    {
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
    }
    else
    {
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
    }
}

void TurnOffPump(void)
{
    Set_Ouput_Pump(OFF);
}

uint8_t Read_Output_Pump(void)
{
    //return(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6));
    return(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14));
}

//Injector
void Set_Ouput_Injector(uint8_t Value)
{
    if (Value == ON)
    {
				//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
    }
    else
    {
        //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    }
}

void Injector_CMD(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm);
}

void TurnOffInjector(void)
{
    Set_Ouput_Injector(OFF);
		Injector_CMD(PWM_0);
}

void Hardware_Init(void)
{
    Set_Output_LED_Green(OFF);
    Set_Output_LED_Red(OFF);
    Set_Output_LED_Blue(OFF);
    Set_Output_LED_Yellow(OFF);
    Set_Ouput_Pump(OFF);
    Set_Ouput_Injector(OFF);
    Injector_CMD(PWM_0);
}
