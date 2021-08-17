/*
 * SENSORS.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "SENSORS.h"
#include "GEN_DEF.h"

uint16_t adcArray[7]={0,0,0,0,0,0,0};                                         //AD converter
volatile sensors_measur sensors={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //All sensor variable related

void Read_Analog_Sensors(void)
{
    sensors.TPSRaw=adcArray[0];
    sensors.TairRaw=adcArray[1];
    sensors.PMapRaw=adcArray[2];
    sensors.EngineTempRaw=adcArray[3];
    sensors.VBatRaw=adcArray[4];
    sensors.VLambdaRaw=adcArray[5];
    sensors.TempBoardRaw=adcArray[6];
}

void LearnTPSLimits(void)
{	
		static uint8_t min_done,max_done;
			
		if(sensors.TPSRaw<scenario.TPS_min)
		{	
				scenario.TPS_min=sensors.TPSRaw;
			
			  //check if I can leave this part...
				if(scenario.TPS_min<600u)
				{	
						scenario.TPS_learned|=1u<<1;	
        }					
		}	
		
		if(sensors.TPSRaw>scenario.TPS_max)
		{	
				scenario.TPS_max=sensors.TPSRaw;
			
				//check if I can leave this part here!
				if(scenario.TPS_max>2000u)
				{	
						scenario.TPS_learned|=1u<<2;
				}	
		}		
		
		min_done=((scenario.TPS_learned&0x02)>>1);
		max_done=((scenario.TPS_learned&0x04)>>2);
		
		if(min_done&&max_done)
		{	
				scenario.TPS_learned|=1u; 
		}	
		else
		{	
				scenario.TPS_learned&=0xFE;		
		}	
}	

void TPSLinearization(void)
{
		//sensors.TPS=(100*(sensors.TPSRaw-tps_min))/(tps_max-tps_min);
	  //sensors.TPSFilt=Filter16bits(sensors.TPSFilt,sensors.TPSRaw,250u);
	  sensors.TPSFilt=sensors.TPSRaw;
	
	  //2000u it was choose just to force learn something close to the real situation
	  if((scenario.TPS_min<600u)||(scenario.TPS_max>2000u))
		{	
				sensors.TPS=(100*(sensors.TPSRaw-scenario.TPS_min))/(scenario.TPS_max-scenario.TPS_min);
				//sensors.TPS=(100*sensors.TPSFilt)/scenario.TPS_max;
		}	
		else
		{	
				sensors.TPS=(100*sensors.TPSFilt)/4095;
		}	
}

void TairLinearization(void)
{
    sensors.Tair=(sensors.TairRaw*150)/4095;
		//sensors.Tair=45;
}

void MAPLinearization(void)
{
    sensors.PMap=(100*sensors.PMapRaw)/4095;
		//sensors.PMap=101;
}

void EngineTempLinearization(void)
{
    sensors.EngineTemp=(sensors.EngineTempRaw*150)/4095;
	  //sensors.EngineTemp=90;
}

void VBatLinearization(void)
{
    //Needs to apply a filter because the real circuit doesn´t have one...
    sensors.VBatFilt=Filter16bits(sensors.VBatFilt,sensors.VBatRaw,100u);
    sensors.VBat=(uint8_t)(((sensors.VBatFilt*347*455)/(4095*1000))+4);
}

void LambdaLinearization(void)
{
    sensors.Lambda=(sensors.VLambdaRaw*100)/85;
	  sensors.Lambda=(100*sensors.VLambdaRaw)/4095;
}

void Board_Temp(void)
{
    //Needs to apply a filter due the sensor characteristics
    sensors.TempBoardFilt=Filter16bits(sensors.TempBoardFilt,sensors.TempBoardRaw,80u);
    sensors.TempBoard=((V25-sensors.TempBoardFilt)/Avg_Slope)+25;
}
