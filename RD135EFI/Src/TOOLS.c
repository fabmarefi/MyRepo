/*
 * TOOLS.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "TOOLS.h"

uint32_t Saturation(uint32_t var,uint32_t sat)
{
    if(var>sat)
    {
        var=sat;
    }

    return(var);
}

uint8_t Filter8bits(uint8_t varOld,uint8_t var,uint8_t k)
{
    int16_t varFiltered;

    varFiltered=(int16_t)varOld-(int16_t)var;
    varFiltered=(int16_t)var+((int16_t)(((int16_t)k*varFiltered)/255));  

    return((uint8_t)varFiltered);
}

uint16_t Filter16bits(uint16_t varOld,uint16_t var,uint8_t k)
{
    int32_t varFiltered;
	  
    varFiltered=(int32_t)varOld-(int32_t)var;
    varFiltered=(int32_t)var+((int32_t)(((int32_t)k*varFiltered)/255));    
	
    return((uint16_t)varFiltered);
}

// A iterative binary search function. It returns 
// location of x in given array arr[l..r] if present, 
// otherwise -1 
int8_t binarySearch(uint8_t arr[],uint8_t l,uint8_t r,uint8_t x) 
{ 
	  uint8_t m;		
	
	  if(x<arr[l])
		{	
			m=l;
			return -1;
		}	
		else if(x>arr[r])
		{	
			m=r;
			return -1;
		}	
		
		r=r-1;
		
	  while(l <= r) 
	  {
				m = l + (r - l) / 2;
  
        // Check if x is present at mid
        //if (arr[m] == x)
			  if ((x>=arr[m])&&(x<arr[m+1])) 
            return m;
  
        // If x greater, ignore left half
        if (arr[m] < x)
            l = m + 1;
  
        // If x is smaller, ignore right half
        else
            r = m - 1;			
	 }	

	 //Error
	 return (0xFE);
} 

//This function was prepared to return a 8 bits value, however is saturated  in 64
//Its mandatory in rpm array there are some difference value between two adjacent fields, if do not respect will cause an error return 0xFF
uint16_t LinInterp16(uint8_t value,uint8_t x_array[],uint16_t y_array[])
{
		int8_t interp_index;
		uint16_t interp_res;
	
		interp_index = binarySearch(x_array, 0, 7, value);	
	
		if(interp_index != -1)
		{
				interp_res = (((y_array[interp_index+1]-y_array[interp_index])*(value-x_array[interp_index]))/(x_array[interp_index+1]-x_array[interp_index]))+y_array[interp_index];    		
				return(interp_res);
		}
		else
		{
				//Advance saturation for array min and max
				if(value<x_array[0])
				{
						return(y_array[0]);
				}
				else if(value>x_array[7])
				{
						return(y_array[7]);
				}
				else
				{	
						return(0xFF);     //return an error value...
				}	
		} 	
}

uint8_t LinInterp8(uint8_t value,uint8_t x_array[],uint8_t y_array[])
{
		int8_t interp_index;
		uint16_t interp_res;
	
		interp_index = binarySearch(x_array, 0, 7, value);
	
		if(interp_index != -1)
		{
				interp_res = (((y_array[interp_index+1]-y_array[interp_index])*(value-x_array[interp_index]))/(x_array[interp_index+1]-x_array[interp_index]))+y_array[interp_index];    		
				return(interp_res);
		}
		else
		{
				//Advance saturation for array min and max
				if(value<x_array[0])
				{
						return(y_array[0]);
				}
				else if(value>x_array[7])
				{
						return(y_array[7]);
				}
				else
				{	
						return(0xFF);     //return an error value...
				}	
		} 	
}
