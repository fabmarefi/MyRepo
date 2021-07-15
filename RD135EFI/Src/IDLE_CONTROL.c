/*
 * IDLE_CONTROL.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

//#define IDLE_CONTROL_c

//#include "ENGINE_STATE.h"
#include "IDLE_CONTROL.h"
//#include "GEN_DEF.h"
//#include "SENSORS.h"
//#include "IO_CONTROL.h"

void pid_Init(void)
{
		pid_control.CumError=0;
		pid_control.Pwm_PI=0;
		pid_control.Pwm_OUT=0;
		pid_control.error_visual=0;
}

void Crank_Pos_IACV(void)
{
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pid_control.Pwm_OUT);
}

void Open_IACV(void)
{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,2069);
}

void Close_IACV(void)
{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,69);
}

uint16_t funcIdleSetpoint(uint8_t temp)
{
		//Here I need to create an array Engine Speed Target in function of Engine Temperature
		pid_control.Engine_Speed_Setpoint=1300;
		return(pid_control.Engine_Speed_Setpoint);
}

void IACV_Control(void)
{
		int32_t Error=0;

		Error=funcIdleSetpoint(sensors.EngineTemp)-scenario.Engine_Speed;
		pid_control.error_visual=Error;

		if((pid_control.Pwm_PI>=0)&&(pid_control.Pwm_PI<=2800u))
		{
				pid_control.CumError+=Error;
		}

		pid_control.Pwm_PI=(((pid_control.kpnum*pid_control.kidenum*Error)+(pid_control.kinum*pid_control.kpdenum*pid_control.CumError))/(pid_control.kpdenum*pid_control.kidenum));

		if((pid_control.Pwm_PI>=0)&&(pid_control.Pwm_PI<=2800u))
		{
				pid_control.Pwm_OUT=pid_control.Pwm_PI;
		}
		else
		{
				pid_control.Pwm_OUT=0u;
		}

		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pid_control.Pwm_OUT);
}

void Learn_test_IACV(void)
{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
}

void Idle_Management(void)
{
		static uint8_t enable_pid=OFF;

		if(sensors.TPS<min_tps_IDLE)
		{
				switch(scenario.Engine_State)
				{
						case WAKEUP:
						case PRIMERINJ: Learn_test_IACV();

						case STOP:
						case CRANK:
						case STALL:			Crank_Pos_IACV();
														enable_pid=1;
														break;

						case IDLE:  		if(enable_pid)
														{
																enable_pid=OFF;
																pid_Init();
														}

														IACV_Control();
														break;

						case CRUISE:
            case OVERSPEED: Open_IACV();
														enable_pid=1;
														break;

						default:       	break;
				}
		}
}
