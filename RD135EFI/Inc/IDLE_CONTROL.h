/*
 * IDLE_CONTROL.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_IDLE_CONTROL_H_
#define INC_IDLE_CONTROL_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"
#include "SENSORS.h"

#define min_tps_IDLE                  30
#define idle_min                    1100
#define idle_max                    1800

//PID control
typedef struct pid_control
{
    uint16_t Engine_Speed_Setpoint;
    uint8_t  error;
    int32_t  CumError;
    uint16_t kpnum;
    uint16_t kpdenum;
    uint16_t kinum;
    uint16_t kidenum;
    int32_t  Pwm_PI;
    uint16_t Pwm_OUT;
		int32_t  error_visual;
}pid_vars;

extern volatile pid_vars pid_control;
extern TIM_HandleTypeDef htim4;

uint16_t funcIdleSetpoint(uint8_t temp);
void Idle_Management(void);

#endif /* INC_IDLE_CONTROL_H_ */
