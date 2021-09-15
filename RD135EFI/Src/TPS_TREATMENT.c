/*
 * TPS_TREATMENT.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "TPS_TREATMENT.h"

//Throttle position sensor treatment
uint8_t funcfastEnrichment(uint8_t TPS)
{
    return(120);
}

uint8_t funcfastEnleanment(int8_t TPS)
{
    return(80);
}

/* Gas treatment */
//create a automatic learning to get tps_min and tps_max
void TPS_Treatment(void)
{
    scenario.deltaTPS=sensors.TPS-scenario.TPS_old;

    if(scenario.deltaTPS>quickCmdPos)
    {
        //Enrichment fuel based in a table deltaTPS(temp)
        Set_Output_LED_Blue(ON);
        scenario.fastEnrichmentTerm=funcfastEnrichment(scenario.deltaTPS);   //1,2
        Cond5=TRUE;
    }
    else if(scenario.deltaTPS<quickCmdNeg)
    {
        //En-leanment fuel based in a table deltaTPS(temp)
				Set_Output_LED_Red(ON);
        scenario.fastEnrichmentTerm=funcfastEnleanment(scenario.deltaTPS);   //0,8
        Cond6=TRUE;
    }

    if(Timeout_ms(Cond5,&Counter5,tfastenrich))
    {
        Cond5=FALSE;
        Set_Output_LED_Blue(OFF);
        scenario.fastEnrichmentTerm=100;
    }
    else if(Timeout_ms(Cond6,&Counter6,tfastenleanment))
    {
        Cond6=FALSE;
			  Set_Output_LED_Red(OFF);
        scenario.fastEnrichmentTerm=100;
    }

    //Pay attention, this function can overwrite enrichment function...
    if(Timeout_ms((sensors.TPS<tps_min_cutoff),&Counter7,TimeToGetInCutoff)
		&&(scenario.Engine_Speed>idle_max))
    {
				scenario.cuttOffTerm=0;
    }
    else
    {
        scenario.cuttOffTerm=1;
    }

    scenario.TPS_old=sensors.TPS;
}
