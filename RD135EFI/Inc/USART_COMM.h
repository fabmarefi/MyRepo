/*
 * USART_COMM.h
 *
 *  Created on: 31-Aug-2021
 *      Author: Jerena
 */

#ifndef INC_USART_COMM_H_
#define INC_USART_COMM_H_

#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"

////UART Communication
extern UART_HandleTypeDef huart3;
extern uint8_t UART3_txBuffer[blockSize+2];
extern uint8_t UART3_rxBuffer[blockSize+2];
extern uint8_t UART3_rxBufferAlt[11];

void initializeCalibOnRAM(void);
void copyCalibUartToRam(void);
void copyCalibRamToUart(void);
void saveCalibRamToFlash(void);
void copyCalibFlashToRam(void);
void transmitCalibToUART(void);
void receiveData(void);
void systemInitialization(void);
void transmitSystemInfo(void);

#endif /* INC_USART_COMM_H_ */
