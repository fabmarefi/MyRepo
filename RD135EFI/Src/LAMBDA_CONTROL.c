/*
 * LAMBDA_CONTROL.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

//#define LAMBDA_CONTROL_c

#include "LAMBDA_CONTROL.h"
//#include "TIMER_FUNC.h"
//#include "SENSORS.h"

/*
Important requirements
Practically, when the engine is in a steady state, fuel mixture deviates from the stoichiometric
in range ±2% ~ ±3% with frequency 1 ~ 2 times per second.
This process can be assessed very well by observing the output signal waveforms of the oxygen sensor.
Transition time of the output voltage should not exceed 120mS from one level to another.
*/

//This control can be used only with narrow band sensor lambda

void LambdaCorrectionFunc(uint8_t lambdaRequested, uint8_t lambdaMeasured)
{
		static enum Lambda crtlStateOld=RICH;

		//Cond5 -> Enrichment // Cond6 -> Enleanment
		if((!scenario.EnabLambdaCtrl)||((Cond5)||(Cond6)))
		{
				return;
		}

    if(sensors.Lambda>lambdaVoltRichTheshould)
    {
        scenario.LambDir=RICH;
    }
    else if(sensors.Lambda<lambdaVoltLeanTheshould)
    {
        scenario.LambDir=LEAN;
    }
    else
    {
        scenario.LambDir=INACTIVE;
    }

    if((lambdaRequested!=Lambda_Stoichiometric)||(scenario.LambDir==INACTIVE))
    {
        //Neutral correction
        scenario.lambdaCorrectTerm=100;
        return;
    }

		if(scenario.LambDir!=crtlStateOld)
		{
				scenario.lambdaCorrectTerm=100;
		}

    //Proportional control
    switch(scenario.LambDir)
    {
				case RICH:      if(scenario.lambdaCorrectTerm<=90)
                        {
														if(scenario.counterNeg==0)
                            {
																scenario.counterNeg=10;
                                scenario.lambdaCorrectTerm=100;
                            }
														else
														{
																scenario.counterNeg--;
														}
                        }
												else
												{
														scenario.lambdaCorrectTerm=scenario.lambdaCorrectTerm-decrement;
												}

                        break;

        case LEAN:      if(scenario.lambdaCorrectTerm>=110)
                        {
                            if(scenario.counterPos==0)
                            {
																scenario.counterPos=10;
                                scenario.lambdaCorrectTerm=100;
                            }
														else
														{
																scenario.counterPos--;
														}
                        }
												else
												{
														scenario.lambdaCorrectTerm=scenario.lambdaCorrectTerm+increment;
												}

                        break;

        default:    		break;
    }

		crtlStateOld=scenario.LambDir;
}
