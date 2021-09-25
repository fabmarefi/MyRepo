/*
 * ENGINE_STATE.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */
 
#include "ENGINE_STATE.h"

void EngineSpeedCalculation(void)
{
    Set_Ouput_InterruptionTest();

    scenario.Measured_Period+=scenario.nOverflow_RE*TMR2_16bits;

    //Engine speed must be greater than 100rpm and less than 15000rpm to consider Measured_Period useful for calculations
    if((scenario.Measured_Period<=EngineSpeedPeriod_Min)&&
       (scenario.Measured_Period>=EngineSpeedPeriod_Max))
    {
        scenario.Engine_Speed=RPM_const/scenario.Measured_Period;
        scenario.Engine_Speed_old=scenario.Engine_Speed;
        scenario.deltaEngineSpeed=scenario.Engine_Speed-scenario.Engine_Speed_old;

        //Linear prediction
        if((scenario.Engine_Speed<<1u)>scenario.Engine_Speed_old)
        {
						scenario.engineSpeedPred=(scenario.Engine_Speed<<1u)-scenario.Engine_Speed_old;
            //scenario.tdutyInputSignalPredLinear = (RPM_const*calibFlashBlock.Calibration_RAM.sensorAngDisplecement)/(scenario.engineSpeedPred*360u);
        }
        else
        {
            scenario.engineSpeedPred=0u;
        }

        //For calculus purpose I decided to use the linear prediction
        scenario.Engine_Speed=scenario.engineSpeedPred;

        scenario.TDuty_Input_Signal+=scenario.nOverflow_FE*TMR2_16bits;
    }

    Set_Ouput_InterruptionTest();
}

void Engine_STOP_test(void)
{
    static uint8_t program=FALSE;
    static uint32_t initial_value;

    if(program==FALSE)
    {
        initial_value=scenario.Rising_Edge_Counter;
        program=TRUE;
    }
    else
    {
        if(scenario.Rising_Edge_Counter==initial_value)
        {
            scenario.Rising_Edge_Counter=0u;
            scenario.Engine_Speed=0u;
            program=FALSE;
        }
    }
}

uint32_t PrimerPulse(uint8_t temp)
{
    uint32_t pulseLength;
	
	  pulseLength=LinInterp32(temp,calibFlashBlock.Calibration_RAM.BP_Engine_Temperature,calibFlashBlock.Calibration_RAM.TB_PrimerPulse);
	  
    return(pulseLength);
}

//Detect engine acceleration and classify as: Pos, Neg or stable
void AccelDer(void)
{
    scenario.deltaEngineSpeed=scenario.Engine_Speed-scenario.Engine_Speed_old;

    if(scenario.deltaEngineSpeed>accel_rate)
    {
        scenario.Acceleration=ACCEL;
    }
    else if(scenario.deltaEngineSpeed<decel_rate)
    {
        scenario.Acceleration=DECEL;
    }
    else
    {
        scenario.Acceleration=STABLE;
    }

    scenario.Engine_Speed_old=scenario.Engine_Speed;
}

//Will running periodically with period equal 50ms (20 times per second)
//This state machine define the engine state
void Eng_State(void)
{
		switch(scenario.Engine_State)
		{
				case WAKEUP:    ////Pump turn on
												if(Read_Output_Pump()==OFF)
												{
														Set_Ouput_Pump(ON);
														Cond0=TRUE;
												}

												//Wait some time to fill fuel rail
												if(Timeout_ms(Cond0,&Counter0,2000))
												{
														Cond0=FALSE;
														Set_Ouput_Injector(ON);
														Injector_CMD(PWM_100);
														setTimeoutHookUp(timerList,Timer0,PrimerPulse(sensors.EngineTemp),&TurnOffInjector);
														scenario.Engine_State=PRIMERINJ;
												}

												break;

				case PRIMERINJ: if(checkTimeoutHookUp(timerList,Timer0))
												{
														scenario.Engine_Speed=0;
														Set_Output_LED_Green(ON);   //Crank allowed
														scenario.Engine_State=STOP;
												}

												break;

				case STOP:      //I need to treat this statment
												if(scenario.pulseDetected)
												{
														Cond2=TRUE;   //Enable timer, timer limit to launch engine
														scenario.Engine_State=CRANK;
												}

												break;

				case CRANK:     if(scenario.Engine_Speed>idle_min)
												{
														//Enable the counter
														Cond3=TRUE;
														//Check if timer is elapsed
														if(Timeout_ms(Cond3,&Counter3,3000))
														{
																Cond3=FALSE;
																Counter3=0;
																Set_Output_LED_Green(OFF);
																scenario.Engine_State=IDLE;
														}
											  }
												else if((scenario.Acceleration==DECEL)||(Timeout_ms(Cond2,&Counter2,2000)))
												{
														Cond2=FALSE;
														Counter2=0;
														Cond4=TRUE;
														scenario.Engine_State=STALL;
												}

												break;

				case STALL:			if((scenario.Engine_Speed<rpm_stopped)||(Timeout_ms(Cond4,&Counter4,1500)))
												{
														Cond4=FALSE;
														Counter4=0;
														scenario.EngineDiedCounter++;
														scenario.Engine_Speed=0;
														Set_Output_LED_Green(ON);   //Crank allowed
														scenario.Engine_State=STOP;
												}

												break;

				case IDLE:      if(scenario.Engine_Speed>idle_max)
												{
														scenario.Engine_State=CRUISE;
												}
												else if(scenario.Engine_Speed<=idle_min)
												{
														scenario.Engine_State=STALL;
														Cond4=TRUE;
												}

												if(scenario.pulseDetected==0)
												{
														scenario.Engine_Speed=0;
														Set_Output_LED_Green(ON);   //Crank allowed
														scenario.Engine_State=STOP;
												}

												break;

				case CRUISE:    if(scenario.Engine_Speed>rpm_max)
												{
														scenario.Engine_State=OVERSPEED;
												}
												else if(scenario.Engine_Speed<=idle_max)
												{
														scenario.Engine_State=IDLE;
												}

												if(scenario.pulseDetected==0)
												{
														scenario.Engine_Speed=0;
														Set_Output_LED_Green(ON);   //Crank allowed
														scenario.Engine_State=STOP;
												}

												break;

				case OVERSPEED: if(scenario.Engine_Speed<=(rpm_max-hyst))
												{
														scenario.Engine_State=CRUISE;
												}

												if(scenario.pulseDetected==0)
												{
														scenario.Engine_Speed=0;
														Set_Output_LED_Green(ON);   //Crank allowed
														scenario.Engine_State=STOP;
												}

												break;

				default:       	break;
	  }
}
