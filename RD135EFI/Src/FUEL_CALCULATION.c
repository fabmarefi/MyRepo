/*
 * FUEL_CALCULATION.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "FUEL_CALCULATION.h"

uint16_t funcCycles(uint8_t temp)
{
    static uint16_t resp;
	
	  resp=LinInterp16(temp,calibFlashBlock.Calibration_RAM.BP_Engine_Temperature,calibFlashBlock.Calibration_RAM.TB_Cycles);
	  	
    return (resp);
}

uint8_t funcwarmUp(uint8_t temp)
{
	  uint8_t resp;
	
	  resp=LinInterp8(temp,calibFlashBlock.Calibration_RAM.BP_Engine_Temperature,calibFlashBlock.Calibration_RAM.TB_WarmUp);
	  	
    return (resp);
}

void WarmUp_Treatment(void)
{
		static uint16_t cycle_limit;
	
	  cycle_limit=funcCycles(sensors.EngineTemp);
			
    if(scenario.counterCycles<cycle_limit)		
    {
        scenario.warmUpTerm=funcwarmUp(sensors.EngineTemp);
    }
    else
    {
        scenario.warmUpTerm=100;
    }
}

void Overspeed_Treatment(void)
{
    if(scenario.Engine_State!=OVERSPEED)
    {
        scenario.overspeedTerm=1;
    }
    else
    {
        scenario.overspeedTerm=0;
			  Injector_CMD(PWM_0);
        scenario.PW_us=PWM_0;	
        return;
    }
}  

uint8_t funccrankTerm(uint8_t temp)
{
    uint8_t resp;
	
	  resp=LinInterp8(temp,calibFlashBlock.Calibration_RAM.BP_Engine_Temperature,calibFlashBlock.Calibration_RAM.TB_Crank);
	  	
    return (resp);
}

uint8_t funcVoleff(uint8_t PMap,uint16_t Engine_Speed)
{
    return (100);
}

uint8_t funcLambda(uint8_t PMap,uint16_t Engine_Speed)
{
    return (100);
}

void InjectorDeadTimeCalc(void)
{
		scenario.InjectorDeadTime=calibFlashBlock.Calibration_RAM.TB_InjectorDeadTime[0];
}

void FuelCalc(void)
{
    if(scenario.Engine_State==STOP)
    {
				Injector_CMD(PWM_0);
        scenario.PW_us=PWM_0;	
    }
    else if(scenario.Engine_State==CRANK)
    {
        //Fuel strategy
        if(sensors.TPS>maxPedalOnCrank)
        {
						Injector_CMD(PWM_0);
            scenario.PW_us=PWM_0;									  
        }
        else
        {
						/*
						scenario.PMap=101;
						scenario.Displacement=135;
						scenario.Tair=45;
						scenario.VoleffCrank=100;
						scenario.crankTerm=100;
						scenario.LambdaRequestedCrank=100;
						scenario.AFRstoich=132;
						scenario.Engine_Speed=800;
						*/

            //After achieve Engine.State=Crank Enable to inject fixed fuel amount
            scenario.Airmass=(10*sensors.PMap*scenario.Displacement*scenario.VoleffCrank*funccrankTerm(sensors.EngineTemp))/((R_div_Mair*(sensors.Tair+273)));
						scenario.Airmass=(scenario.Airmass*scenario.Engine_Speed)/3000;
						scenario.Fuelmass=(10000*scenario.Airmass)/(scenario.AFRstoich*scenario.LambdaRequestedCrank);
						scenario.Injectormassflow=scenario.Fuelmass;
						scenario.PW_percent=scenario.Injectormassflow/375;
						scenario.PW_us=(scenario.PW_percent*InjectorMaxTime)/1000;
						scenario.PW_us=Saturation(scenario.PW_us,InjectorMaxTime);
						Injector_CMD(scenario.PW_us);
        }
    }
    else if(scenario.Engine_State>CRANK)
    {
        WarmUp_Treatment();
        TPS_Treatment();
        Overspeed_Treatment();

        scenario.Voleff=funcVoleff(sensors.PMap,scenario.Engine_Speed);
        scenario.LambdaRequested=funcLambda(sensors.PMap,scenario.Engine_Speed);
        LambdaCorrectionFunc(scenario.LambdaRequested,sensors.Lambda);

				/*
        scenario.PMap=101;
        scenario.Displacement=135;
        scenario.Tair=45;
        scenario.Voleff=100;
        scenario.warmUpTerm=100;
        scenario.fastEnrichmentTerm=100;
        scenario.crankTerm=100;
        scenario.lambdaCorrectTerm=100;
				scenario.LambdaRequested=100;
        scenario.AFRstoich=132;
        scenario.Engine_Speed=3000;
        */

				scenario.Airmass=(1000*sensors.PMap*scenario.Displacement*scenario.Voleff)/((R_div_Mair*(sensors.Tair+273)));
				scenario.Airmass=(scenario.Airmass*scenario.Engine_Speed)/3000;
				scenario.Fuelmass=(10000*scenario.Airmass)/(scenario.AFRstoich*scenario.LambdaRequested);
				scenario.TotalTerm=(scenario.warmUpTerm*scenario.fastEnrichmentTerm*scenario.lambdaCorrectTerm*scenario.cuttOffTerm*scenario.overspeedTerm)/1000;
				scenario.Injectormassflow=(scenario.Fuelmass*scenario.TotalTerm)/1000;
				scenario.PW_percent=scenario.Injectormassflow/375;
				scenario.PW_us=(scenario.PW_percent*InjectorMaxTime)/1000;
				scenario.PW_us=Saturation(scenario.PW_us,InjectorMaxTime);
				Injector_CMD(scenario.PW_us);
    }
}
