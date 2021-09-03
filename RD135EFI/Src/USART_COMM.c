/*
 * USART_COMM.c
 *
 *  Created on: 31-Aug-2021
 *      Author: Jerena
 */

#include "USART_COMM.h"

//Local definition (if I want to share this variables with another modules, I need to include in header file extern + variable name
/*****************************/
/*                           */
/* -Defines                  */
/* -Variables                */
/* -Initial settings         */
/*                           */
/*****************************/

uint8_t flgTransmition = OFF;
enum Transmission_Status {TRANSMITING, TRANSMISSION_DONE} transmstatus;
enum Reception_Status {DATA_AVAILABLE_RX_BUFFER, RECEPTION_DONE} receptstatus;
uint8_t UART3_txBuffer[blockSize+2];
uint8_t UART3_rxBuffer[blockSize+2];
uint8_t UART3_rxBufferAlt[11];

//These function will be available for all modules, but if I don´t declare in header file, compiler will set warning messages
/*****************************/
/*                           */
/*  Function definition      */
/*                           */
/*****************************/

void initializeCalibOnRAM(void)
{
    uint32_t i;

    for(i=0;i<sizeof(struct_Calibration);i++)
    {
			  calibFlashBlock.array_Calibration_RAM_UART[i] = Initial_Calibration.array_Calibration_RAM_UART[i];
    }
}

void copyCalibUartToRam(void)	
{
    uint32_t i;

    for(i=0;i<sizeof(struct_Calibration);i++)
    {
        calibFlashBlock.array_Calibration_RAM_UART[i] = UART3_rxBuffer[i+1];
    }
}

void copyCalibRamToUart(void)
{
    uint8_t i;

    for(i=0;i<blockSize;i++)
    {
        UART3_rxBuffer[i] = calibFlashBlock.array_Calibration_RAM[i];
    }
}

void saveCalibRamToFlash(void)
{
	  Flash_Write_Data (0x0801FC00, calibFlashBlock.array_Calibration_RAM, (sizeof(calibFlashBlock.array_Calibration_RAM))>>2);	  	
}

void copyCalibFlashToRam(void)
{
    Flash_Read_Data (0x0801FC00, calibFlashBlock.array_Calibration_RAM);
}

void transmitCalibToUART(void)
{
    uint32_t i;
    uint8_t checksum;
    uint32_t buffer_length;

    if(transmstatus == TRANSMISSION_DONE)
    {
        buffer_length = sizeof(UART3_txBuffer);

        UART3_txBuffer[0] = 0x7E;
        checksum = UART3_txBuffer[0];

        for (i=1;i<buffer_length-2;i++)
        {
            UART3_txBuffer[i] = calibFlashBlock.array_Calibration_RAM_UART[i-1];
            checksum += UART3_txBuffer[i];
        }

        UART3_txBuffer[buffer_length-1] = checksum;
        transmstatus = TRANSMITING;
        HAL_UART_Transmit_DMA(&huart3, UART3_txBuffer, sizeof(UART3_txBuffer));
    }
}

void receiveData(void)
{
    uint8_t command;
    uint8_t checksum;
    uint32_t buffer_length;
    uint32_t i;

    if(receptstatus == DATA_AVAILABLE_RX_BUFFER)
    {
        buffer_length = sizeof(UART3_rxBuffer);

        for(i=0;i<buffer_length-1;i++)
        {
            checksum += UART3_rxBuffer[i];
        }

        if((UART3_rxBuffer[buffer_length-1]-checksum) == 0u)				
        {
            command = UART3_rxBuffer[0];

            switch(command)
            {
								case 0x02:  flgTransmition = ON;
                            break;
							
								case 0x03:  flgTransmition = OFF;
                            break;
							
								case 0x47:  saveCalibRamToFlash();
							              break;
							
                case 0x69:  transmitCalibToUART();
                            break;
							
							  case 0x7E:  copyCalibUartToRam();
                            break;              

                default:    break;
            }
        }

        receptstatus = RECEPTION_DONE;
    }
}

void systemInitialization(void)
{	
	  copyCalibFlashToRam();
	
    //Identify if there are some data recorded
    if (calibFlashBlock.Calibration_RAM.Max_Engine_Speed == 0u)
    {
				initializeCalibOnRAM();  
        saveCalibRamToFlash();      
    }
	  		
    //Communication
    transmstatus = TRANSMISSION_DONE;
    receptstatus = RECEPTION_DONE;
}

void transmitSystemInfo(void)
{
    uint8_t Mil, Cent, Dez, Unid;
    uint16_t Man, num, num1;
	  uint8_t checksum;
	  uint32_t i;

    num = scenario.Engine_Speed;
    num1 = scenario.pulseDetected;  // only for test purpose DELETE!!!

    if((num<=9999u)&&(num1<=999u))
    {
        Mil = (num/1000u)+0x30;
        Man = num%1000u;
        Cent = (Man/100u)+0x30;
        Man = Man%100u;
        Dez = (Man/10u)+0x30;
        Unid = (Man%10u)+0x30;

        UART3_rxBufferAlt[0]='R';
        UART3_rxBufferAlt[1]=Mil;
        UART3_rxBufferAlt[2]=Cent;
        UART3_rxBufferAlt[3]=Dez;
        UART3_rxBufferAlt[4]=Unid;
        UART3_rxBufferAlt[5]='A';

        Cent = (num1/100u)+0x30;
        Man = num1%100u;
        Dez = (Man/10u)+0x30;
        Unid = (Man%10u)+0x30;

        UART3_rxBufferAlt[6]=Cent;
        UART3_rxBufferAlt[7]=Dez;
        UART3_rxBufferAlt[8]=Unid;        

        for(i=0; i < sizeof(UART3_rxBufferAlt)-3; i++)
				{
						checksum += UART3_rxBufferAlt[i];
				}	
				
				UART3_rxBufferAlt[9]=checksum; 
				UART3_rxBufferAlt[10]=0x0A; // '\n' - Line feed

        transmstatus = TRANSMITING;
        HAL_UART_Transmit_DMA(&huart3, UART3_rxBufferAlt, sizeof(UART3_rxBufferAlt));
		}		
}
