/*
 * VRS_TREATMENT.c
 *
 *  Created on: 24-Jul-2021
 *      Author: Jerena
 */
 
#include "VRS_TREATMENT.h"

void PulseDetection(void)
{
	  static uint32_t CounterPulseNow,CounterPulseOld;

		Cond8=TRUE;

	  CounterPulseNow=scenario.Rising_Edge_Counter;

		if((CounterPulseNow-CounterPulseOld)>=1)
		{
				scenario.pulseDetected=1;
		}
		else
		{
			  if(Timeout_ms(Cond8,&Counter8,600))
				{
						Cond8=FALSE;
						scenario.pulseDetected=0;
				}
		}

		CounterPulseOld=CounterPulseNow;
}

void Rising_Edge_Event(void)
{
    scenario.Measured_Period = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
    scenario.nOverflow_RE = scenario.nOverflow;
    __HAL_TIM_SET_COUNTER(&htim2,0u);
    scenario.nOverflow = 0u;
    scenario.Rising_Edge_Counter++;

		if (scenario.Rising_Edge_Counter>=2)
		{
				scenario.Update_calc = 1;        //set zero after Engine Stop was detected
		}
}

void Falling_Edge_Event(void)
{
    scenario.TDuty_Input_Signal = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
    scenario.nOverflow_FE = scenario.nOverflow;

    if (scenario.Rising_Edge_Counter>=2u)
    {
        scenario.Update_calc = TRUE;        //set zero after Engine Stop was detected
    }
}
