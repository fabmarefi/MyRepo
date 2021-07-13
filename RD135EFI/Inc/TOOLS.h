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
uint8_t Filter8bits(uint8_t varOld,uint8_t var, uint8_t k);
uint16_t Filter16bits(uint16_t varOld,uint16_t var,uint8_t k);
int8_t binarySearch(uint16_t arr[], uint8_t l, uint8_t r, uint16_t x);
uint8_t Linear_Interpolation(uint16_t value, uint16_t x_array[], uint8_t y_array[]);
//Test
int8_t caraio(volatile uint8_t arr[], uint8_t l, uint8_t r, uint8_t x);
uint16_t buceta(uint8_t value,volatile uint8_t x_array[],volatile uint16_t y_array[]);
int8_t morcega(volatile uint8_t arr[], uint8_t l, uint8_t r, uint8_t x);
uint8_t panceta(uint8_t value, volatile uint8_t x_array[], volatile uint8_t y_array[]);

#endif /* INC_TOOLS_H_ */
