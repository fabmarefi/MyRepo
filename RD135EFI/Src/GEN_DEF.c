/*
 * GEN_DEF.c
 *
 *  Created on: 13-Jul-2021
 *      Author: Jerena
 */

#include "GEN_DEF.h"

volatile struct_Calibration Calibration_RAM = {8000,
	                                            ////The first Engine Speed value in the array needs to be 1200 mandatory
                                              {1200, 2000, 3000, 3500, 4500, 5000, 6000, 7000},
																							{  10,   30,   45,   55,   80,  100,  120,  150},
																							{  64,   64,   64,   64,   64,   64,   64,   64},
																							{  64,   64,   64,   64,   64,   64,   64,   64},
																							{ 200,  170,  160,  150,  140,  120,  105,   50},
																							{ 500, 1000, 1500, 2500, 4000,15000,25000,60000},
																							{  50,   60,   70,   90,  100,  150,  250,  300},
																							{ 145,  135,  125,  115,  110,  100,   95,   90},
																						  {{ 50,  100,  100,  100,  100,  100,  100,  100},
																						  {  45,  100,  100,  100,  100,  100,  100,  100},
																						  {  40,  100,  100,  100,  100,  100,  100,  100},
																						  {  30,  100,  100,  100,  100,  100,  100,  100},
																						  {  25,  100,  100,  100,  100,  100,  100,  100},
																						  {  20,  100,  100,  100,  100,  100,  100,  100},
																						  {  15,  100,  100,  100,  100,  100,  100,  100},
																						  {   5,  100,  100,  100,  100,  100,  100,  100}},
                                             {{  90,   90,   90,   90,   90,   90,   90,   90},
																						  { 100,  100,  100,  100,  100,  100,  100,  100},
																						  { 100,  100,  100,  100,  100,  100,  100,  100},
																						  { 100,  100,  100,  100,  100,  100,  100,  100},
																						  { 100,  100,  100,  100,  100,  100,  100,  100},
																						  { 100,  100,  100,  100,  100,  100,  100,  100},
																						  { 100,  100,  100,  100,  100,  100,  100,  100},
																						  { 100,  100,  100,  100,  100,  100,  100,  100}}};

volatile system_vars scenario={WAKEUP,STABLE,1,0,0,0,0,0,0,0,0,0,0,0,0,0,RICH,TRUE,100,100,0,100,40,135,132,50,0,0,0,0,0,0,0,10,10,0,0,0,1,100,100,100,100,100,100};		
	