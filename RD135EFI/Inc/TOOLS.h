/*
 * TOOLS.h
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#ifndef INC_TOOLS_H_
#define INC_TOOLS_H_

#include "stm32f1xx_hal.h"

uint32_t Saturation(uint32_t var,uint32_t sat);
uint8_t Filter8bits(uint8_t varOld,uint8_t var,uint8_t k);
uint16_t Filter16bits(uint16_t varOld,uint16_t var,uint8_t k);
int8_t binarySearch(uint8_t  arr[],uint8_t l,uint8_t r,uint8_t  x);
uint8_t LinInterp8(uint8_t  value,uint8_t  x_array[],uint8_t  y_array[]);
uint16_t LinInterp16(uint8_t  value,uint8_t  x_array[],uint16_t y_array[]);

#endif /* INC_TOOLS_H_ */
