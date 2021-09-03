/*
 * USART_COMM.h
 *
 *  Created on: 31-Aug-2021
 *      Author: Jerena
 */

#ifndef INC_USART_COMM_H_
#define INC_USART_COMM_H_

/*Include to use in my code*/
#include "stm32f1xx_hal.h"
#include "GEN_DEF.h"
#include "FLASH_PAGE.h"

////UART Communication
//Will be available for all modules that include USART_COMM.h file...
extern uint8_t flgTransmition;

/*
If I put extern in front of the variable that belongs to c file, another modules can access this variables
*/

//This is a special case, It was declared in main.c, but I will use in USART_COMM.c and if another module include USART_COMM.h
//will be available this variable to be use it, like an apropriation...
extern UART_HandleTypeDef huart3;

//In this case, all modules that include USART_COMM.h, these functions will be available, if they aren´t declared 
//compiler wil set a warning message 
void receiveData(void);
void systemInitialization(void);
void transmitSystemInfo(void);

#endif /* INC_USART_COMM_H_ */
