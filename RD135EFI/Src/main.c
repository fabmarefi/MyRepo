/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FLASH_PAGE.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
#define FALSE                          0
#define TRUE                           1
#define OFF                            0
#define ON                             1
#define nTimer                        16
#define TMR2_16bits               65536u
#define EngineSpeedPeriod_Min    785455u     //100rpm
#define EngineSpeedPeriod_Max      5236u     //15000rpm
#define RPM_const              78545455u
#define PWM_0                          0
#define PWM_100                     1001

#define AFR_Bensine                  132
#define AFR_Ethanol                   90
#define Lambda_Stoichiometric        100
#define accel_rate                   150
#define decel_rate                  -150
#define maxPedalOnCrank               70
#define rpm_max                     7000
#define hyst                         300
#define rpm_stopped                  300
#define idle_min                    1100
#define idle_max                    1800
#define quickCmdPos                   30
#define quickCmdNeg                  -30
#define threshould                  2500
#define tfastenrich                 1000
#define tfastenleanment              200
#define tps_min_cutoff                20
#define increment                      1
#define decrement                      1
#define min_tps_IDLE                  30
#define V25                          180
#define Avg_Slope                      5
#define tps_min                       30
#define tps_max                      190
#define lambdaVoltLeanTheshould       30
#define lambdaVoltRichTheshould       70
#define InjectorMaxTime              850   //85% time

uint8_t Cond0=0,Cond1=0,Cond2=0,Cond3=0,Cond4=0,Cond5=0,Cond6=0;
uint32_t Counter0=0,Counter1=0,Counter2=0,Counter3=0,Counter4=0,Counter5=0,Counter6=0,Counter7=0,Counter8=0,Counter9=0,Counter10=0;
uint8_t pulseDetected=0;

//Scheduller
typedef struct Scheduler
{
    uint8_t  program;
    uint32_t target_time;
}sched_var;

sched_var array_sched_var[3];

enum TimerID{Timer0,Timer1,Timer2,Timer3,Timer4,Timer5,Timer6,Timer7,Timer8,Timer9,Timer10,Timer11,Timer12,Timer13,Timer14,Timer15};// TimerNumb;
enum EngineState{WAKEUP,PRIMERINJ,STOP,CRANK,STALL,IDLE,CRUISE,OVERSPEED};
enum Accel{ACCEL,DECEL,STABLE};
enum Lambda{RICH,LEAN,INACTIVE};


typedef struct TimerStruct
{
    uint32_t target_time;
    uint8_t  output;
    void (*func_pointer)();
}timerSchedtype;

timerSchedtype timerList[nTimer]; //={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
 
typedef struct system_info
{
        enum EngineState Engine_State;         //WAKEUP
        enum Accel Acceleration;               //STABLE
        uint16_t Engine_Speed_old;             //0
        uint16_t Engine_Speed;                 //0   
        int16_t  deltaEngineSpeed;             //0
        uint16_t engineSpeedPred;              //0
        uint16_t engineSpeedFiltered;          //0
        uint16_t avarageEngineSpeed;           //0
        uint32_t Measured_Period;              //0

        uint32_t TDuty_Input_Signal;           //0
        uint32_t tdutyInputSignalPred;         //0
        uint32_t tdutyInputSignalPredLinear;   //0

        int8_t   deltaTPS;                     //0
        uint8_t  TPS_old;                      //0
        enum Lambda LambDir;                   //RICH
        uint8_t  Update_calc;                  //TRUE
        uint8_t  LambdaRequested;              //100

	      uint32_t Airmass;                      //0
        uint8_t  Voleff;                       //100
        uint8_t  Displacement;                 //135
        uint8_t  AFRstoich;                    //132
				uint16_t InjectorDeadTime;             //50  (1ms)
        uint32_t Injectormassflow;             //0
        uint32_t PW_us;                        //0
				uint32_t PW_percent;                   //0
        uint32_t Fuelmass;                     //0

        uint32_t nOverflow;                    //0
        uint8_t  EngineDiedCounter;            //0
        uint32_t counterCycles;                //0
        uint8_t  counterPos;                   //10
        uint8_t  counterNeg;                   //10
        uint32_t nOverflow_RE;                 //0
        uint32_t nOverflow_FE;                 //0
        uint32_t Rising_Edge_Counter;          //0

        uint8_t  cuttOffTerm;                  //1
        uint8_t  fastEnrichmentTerm;           //100
        uint8_t  warmUpTerm;                   //100
        uint8_t  overspeedTerm;                //100
        uint8_t  crankTerm;                    //100
        uint8_t  lambdaCorrectTerm;            //100
				uint8_t  TotalTerm;                    //100

        uint8_t  VBatRaw;                      //0
        uint8_t  VBatFilt;                     //0
        uint8_t  VBat;                         //0

        uint8_t  VLambdaRaw;                   //0
        uint8_t  VLambdaFilt;                  //0
        uint8_t  Lambda;                       //0

        uint8_t  TempBoardRaw;                 //0
        uint8_t  TempBoardFilt;                //0
        uint8_t  TempBoard;                    //0

        uint8_t  TPSRaw;                       //0
        uint8_t  TPSFilt;                      //0
        uint8_t  TPS;                          //0

        uint8_t  PMapRaw;                      //0
        uint8_t  PMapFilt;                     //0
        uint8_t  PMap;                         //0

        uint8_t  TairRaw;                      //0
        uint8_t  TairFilt;                     //0
        uint8_t  Tair;                         //0

        uint8_t  EngineTempRaw;                //0
        uint8_t  EngineTempFilt;               //0
        uint8_t  EngineTemp;                   //0
}system_vars;

volatile system_vars scenario={WAKEUP,STABLE,0,0,0,0,0,0,0,0,0,0,0,0,RICH,TRUE,100,0,100,135,132,50,0,0,0,0,0,0,0,10,10,0,0,0,1,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//PID control
typedef struct pid_control
{
        uint16_t Engine_Speed_Setpoint;
        uint8_t error;
        int32_t CumError;
        uint16_t kpnum;
        uint16_t kpdenum;
        uint16_t kinum;
        uint16_t kidenum;
        int32_t Pwm_PI;
        uint16_t Pwm_OUT;
        int32_t error_visual;
}pid_vars;

volatile pid_vars pid_control={1300,0,0,600,1000,20,1000,0,0,0};

uint16_t InjectorDeadTimeArray[8]={50,60,70,90,100,150,250,300};        

static uint32_t a,b,c,d,e,f,g,h;

/*
PMap=100;    //KPa
Voleff=100;  //from table
Displacement=163;
AFR=132;
Lambda=100;
Tair=308;  //273+35
Injectormassflow=1450;

Fuelmass=(34847/1000000)*((PMap*Voleff*Displacement)/(AFR*Lambda*Tair));
PW_us=(Fuelmass*600000000)/Injectormassflow;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

uint8_t adcArray[7];

//Function prototypes
void TPS_Treatment(void);
void Eng_Status(void);
void AccelDer(void);
void Idle_Management(void);
void FuelCalc(void);
void Read_Analog_Sensors(void);
void Board_Temp(void);
uint8_t digitalFilter8bits(uint8_t var, uint8_t k);
void VBatLinearization(void);
void TPSLinearization(void);
void MAPLinearization(void);
void LambdaLinearization(void);
void EngineTempLinearization(void);
void TairLinearization(void);
void LambdaCorrectionFunc(uint8_t lambdaRequested, uint8_t lambdaMeasured);
uint8_t funcLambda(uint8_t PMap,uint16_t Engine_Speed);
void InjectorDeadTimeCalc(void);
void Injector_CMD(uint16_t pwm);

//Led Green (Bluepill)
void Toggle_LED_Green(void)
{
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}

void Set_Output_LED_Green(uint8_t	Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		}
}

//Led Red
void Toggle_LED_Red(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
}

void Set_Output_LED_Red(uint8_t	Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		}
}

//Led Blue
void Toggle_LED_Blue(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
}

void Set_Output_LED_Blue(uint8_t Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		}
}

//Led Yellow
void Toggle_LED_Yellow(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
}

void Set_Output_LED_Yellow(uint8_t Value)
{
		if(Value==ON)
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		}
		else
		{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		}
}

void Set_Ouput_Pump(uint8_t Value)
{
    if (Value==ON)
    {
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
    }
    else
    {
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
    }
}

uint8_t Read_Output_Pump(void)
{
        //return(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6));
        return(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14));
}

void Set_Ouput_Injector(uint8_t Value)
{
    if (Value == ON)
    {
        //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
    }
    else
    {
        //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    }
}

void Hardware_Init(void)
{
        Set_Output_LED_Green(OFF);
        Set_Output_LED_Red(OFF);
        Set_Output_LED_Blue(OFF);
        Set_Output_LED_Yellow(OFF);
        Set_Ouput_Pump(OFF);
        Set_Ouput_Injector(OFF);
	      Injector_CMD(0);
}

void Set_Ouput_InterruptionTest(void)
{
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
}

void Task_Fast(void)
{
        //HAL_IWDG_Init(&hiwdg);

        //Read analog sensors
        Read_Analog_Sensors();

        //Treat analog sensors
        TPSLinearization();
        MAPLinearization();
        LambdaLinearization();
        EngineTempLinearization();
        TairLinearization();

        //Fuel calculation          	
        AccelDer();
	      InjectorDeadTimeCalc();
        Eng_Status();
        
        FuelCalc();
}

void Task_Medium(void)
{
        Idle_Management();
}

void Task_Slow(void)
{
        Board_Temp();
        VBatLinearization();
}

void Board_Temp(void)
{
        //Needs to apply a filter due the sensor characteristics
        scenario.TempBoardFilt=digitalFilter8bits(scenario.TempBoardRaw,180);
        scenario.TempBoard=((V25-scenario.TempBoardFilt)/Avg_Slope)+25;
}

void VBatLinearization(void)
{
        //Needs to apply a filter besause the real circuit doesn´t have one...
        scenario.VBatFilt=digitalFilter8bits(scenario.VBatRaw,180);
        scenario.VBat=(scenario.VBatFilt*150)/255;
}

void TPSLinearization(void)
{
        scenario.TPS=(100*(scenario.TPSRaw-tps_min))/(tps_max-tps_min);
}

void MAPLinearization(void)
{
        scenario.PMap=(scenario.PMapRaw*110)/255;
	      scenario.PMap=101;
}

void LambdaLinearization(void)
{
        scenario.Lambda=(scenario.VLambdaRaw*100)/85;				
}

void EngineTempLinearization(void)
{
        scenario.EngineTemp=(scenario.EngineTempRaw*180)/255;
}

void TairLinearization(void)
{
        scenario.Tair=(scenario.TairRaw*150)/255;
	      scenario.Tair=45;
}

void InjectorDeadTimeCalc(void)
{
				scenario.InjectorDeadTime=InjectorDeadTimeArray[0];
}	

void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos)
{
    volatile uint32_t counter;

    counter = HAL_GetTick();

    if(var[pos].program == FALSE)
    {
        var[pos].target_time = counter+period;
        var[pos].program = TRUE;
    }

    if(counter>=var[pos].target_time)
    {
        var[pos].program = FALSE;
        (*func)();
    }
}

void Crank_Pos_IACV(void)
{
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pid_control.Pwm_OUT);
}

void Open_IACV(void)
{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,2069);
}

void Close_IACV(void)
{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,69);
}

uint16_t funcIdleSetpoint(uint8_t temp)
{
		//Here I need to create an array Engine Speed Target in function of Engine Temperature
		pid_control.Engine_Speed_Setpoint=1300;
		return(pid_control.Engine_Speed_Setpoint);
}

void pid_Init(void)
{
		pid_control.CumError=0;
		pid_control.Pwm_PI=0;
		pid_control.Pwm_OUT=0;
		pid_control.error_visual=0;
}

void IACV_Control(void)
{
		int32_t Error=0;

		Error=funcIdleSetpoint(scenario.EngineTemp)-scenario.Engine_Speed;
		pid_control.error_visual=Error;

		if((pid_control.Pwm_PI>=0)&&(pid_control.Pwm_PI<=2800u))
		{
				pid_control.CumError+=Error;
		}

		pid_control.Pwm_PI=(((pid_control.kpnum*pid_control.kidenum*Error)+(pid_control.kinum*pid_control.kpdenum*pid_control.CumError))/(pid_control.kpdenum*pid_control.kidenum));

		if((pid_control.Pwm_PI>=0)&&(pid_control.Pwm_PI<=2800u))
		{
				pid_control.Pwm_OUT=pid_control.Pwm_PI;
		}
		else
		{
				pid_control.Pwm_OUT=0u;
		}

		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pid_control.Pwm_OUT);
}

void Learn_test_IACV(void)
{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
}

void Idle_Management(void)
{
		static uint8_t enable_pid=OFF;

		if(scenario.TPS<min_tps_IDLE)
		{
				switch(scenario.Engine_State)
				{
					  case WAKEUP:
						case PRIMERINJ:   Learn_test_IACV();

						case STOP:
						case CRANK:
						case STALL: 			Crank_Pos_IACV();
															enable_pid=1;
															break;

						case IDLE:    		if(enable_pid)
															{
																	enable_pid=OFF;
																	pid_Init();
															}

															IACV_Control();
															break;

						case CRUISE:
            case OVERSPEED: 	Open_IACV();
															enable_pid=1;
															break;

						default:          break;
				}
	  }
}

// A iterative binary search function. It returns
// location of x in given array arr[l..r] if present,
// otherwise -1
uint8_t binarySearch(volatile uint16_t array[], uint8_t first, uint8_t last, uint16_t search)
{
    uint8_t middle;

    middle = (first+last)>>1;

    while (first <= last)
    {
        if((search >= array[middle])&&
           (search <= array[middle+1]))
        {
            return middle;
        }
        else if(search > array[middle+1])
        {
            first = middle+1;
        }
        else //search < array[middle]
        {
            last = middle;
        }

        middle = (first + last)>>1;
    }

    return (255u);
}

//This function was prepared to return a 8 bits value, however is saturated  in 64
//Its mandatory in rpm array there are some difference value between two adjacent fields, if do not respect will cause an error return 0xFF
uint8_t linearInterpolation(uint16_t value, volatile uint16_t x_array[], volatile uint8_t y_array[])
{
    uint8_t interp_index;
    uint8_t interp_res;

    //Advance saturation for array min and max
    if(value<x_array[0])
    {
        return(y_array[0]);
    }
    else if(value>x_array[11])
    {
        return(y_array[11]);
    }

    interp_index = binarySearch(x_array, 0, 11, value);

    if(((x_array[interp_index+1]-x_array[interp_index])!=0)&&(interp_index!=255u))
    {
        interp_res = (((y_array[interp_index+1]-y_array[interp_index])*(value-x_array[interp_index]))/(x_array[interp_index+1]-x_array[interp_index]))+y_array[interp_index];
        if(interp_res>64u)
        {
            interp_res = 64u;
        }
        return(interp_res);
    }
    else
    {
        return(255u);
    }
}

void Engine_STOP_test(void)
{
    static uint8_t program;
    static uint32_t initial_value;

    if(program == FALSE)
    {
        initial_value = scenario.Rising_Edge_Counter;
        program = TRUE;
    }
    else
    {
        if(scenario.Rising_Edge_Counter == initial_value)
        {
            scenario.Rising_Edge_Counter = 0u;
            scenario.Engine_Speed = 0u;
            program = FALSE;
        }
    }
}

uint8_t digitalFilter8bits(uint8_t var, uint8_t k)
{
    static uint8_t varOld = 0u;
    uint8_t varFiltered;

    varFiltered = var + (((varOld-var)*k)/255u);
    varOld = var;

    return(varFiltered);
}

uint16_t digitalFilter16bits(uint16_t var, uint8_t k)
{
    static uint16_t varOld = 0u;
    uint16_t varFiltered;

    varFiltered = var + (((varOld-var)*k)/255u);
    varOld = var;

    return(varFiltered);
}

void setTimeoutHookUp(enum TimerID timer,uint32_t period,void (*func)(void))
{
    timerList[timer].target_time=HAL_GetTick()+period;
    timerList[timer].func_pointer=*func;
    timerList[timer].output=FALSE;
}

uint8_t checkTimeoutHookUp(enum TimerID Timer)
{
    uint8_t TempResp=FALSE;

    if(timerList[Timer].output==TRUE)
    {
        timerList[Timer].output=FALSE;
        timerList[Timer].target_time=0;
        timerList[Timer].func_pointer=0;
        TempResp=TRUE;
    }

    return (TempResp);
}

void TimerListManagement(timerSchedtype timerList[])
{
    uint8_t line;

    for(line=0;line<nTimer;line++)
    {
        if(timerList[line].target_time!=0)
        {
            if(HAL_GetTick()>=timerList[line].target_time)
            {
							  timerList[line].target_time=0;
                timerList[line].output=TRUE;
                timerList[line].func_pointer();
            }
        }
    }
}

uint8_t Timeout_ms(uint8_t Condition,uint32_t *timer,uint32_t period)
{
    uint8_t TimeoutResp=FALSE;

    if(Condition==TRUE)
    {
        if(*timer==0)
        {
            *timer=HAL_GetTick()+period;
        }

        if(HAL_GetTick()>=*timer)
        {
            TimeoutResp=TRUE;
        }
    }
    else
    {
        *timer=0;
    }

    return (TimeoutResp);
}

void TurnOffPump(void)
{
    Set_Ouput_Pump(OFF);
}

void TurnOffInjector(void)
{
    Set_Ouput_Injector(OFF);
	  Injector_CMD(PWM_0);
}

uint32_t PrimerPulse(uint8_t temp)
{
    uint32_t pulseLength;

    if(temp>80)
    {
        pulseLength=150;
    }
    else
    {
        pulseLength=10;
    }

    return(pulseLength);
}

//Detect engine acceleration and classify as: Pos, Neg or stable
void AccelDer(void)
{
    scenario.deltaEngineSpeed=scenario.Engine_Speed-scenario.Engine_Speed_old;

    if(scenario.deltaEngineSpeed>accel_rate)
    {
        scenario.Acceleration=ACCEL;
    }
    else if(scenario.deltaEngineSpeed<decel_rate)
    {
        scenario.Acceleration=DECEL;
    }
    else
    {
        scenario.Acceleration=STABLE;
    }

    scenario.Engine_Speed_old=scenario.Engine_Speed;
}

void Read_Analog_Sensors(void)
{
    scenario.TPSRaw=adcArray[0];
    scenario.TairRaw=adcArray[1];
    scenario.PMapRaw=adcArray[2];
    scenario.EngineTempRaw=adcArray[3];
    scenario.VBatRaw=adcArray[4];
    scenario.VLambdaRaw=adcArray[5];
    scenario.TempBoardRaw=adcArray[6];
}

//Throttle position sensor treatment
uint8_t funcfastEnrichment(uint8_t TPS)
{
    return(120);
}

uint8_t funcfastEnleanment(uint8_t TPS)
{
    return(80);
}

/* Gas treatment */
//create a automatic learning to get tps_min and tps_max
void TPS_Treatment(void)
{
    scenario.deltaTPS=scenario.TPS-scenario.TPS_old;

    if(scenario.deltaTPS>quickCmdPos)
    {
        //Enrichment fuel based in a table deltaTPS(temp)
        Set_Output_LED_Red(ON);
        scenario.fastEnrichmentTerm=funcfastEnrichment(scenario.deltaTPS);   //1,2
        Cond5=TRUE;
    }
    else if(scenario.deltaTPS<quickCmdNeg)
    {
        //En-leanment fuel based in a table deltaTPS(temp)
        scenario.fastEnrichmentTerm=funcfastEnleanment(scenario.deltaTPS);   //0,8
        Cond6=TRUE;
    }

    if(Timeout_ms(Cond5,&Counter5,tfastenrich))
    {
        Cond5=FALSE;
        Set_Output_LED_Red(OFF);
        scenario.fastEnrichmentTerm=100;
    }
    else if(Timeout_ms(Cond6,&Counter6,tfastenleanment))
    {
        Cond6=FALSE;
        scenario.fastEnrichmentTerm=100;
    }

    //Pay attention, this function can overwrite enrichment function...
    if((scenario.TPS<tps_min_cutoff)&&(scenario.Engine_Speed>threshould))
    {
        scenario.cuttOffTerm=0;
    }
    else
    {
        scenario.cuttOffTerm=1;
    }

    scenario.TPS_old=scenario.TPS;
}

uint8_t funcwarmUp(uint8_t temp)
{
    return (130);
}

uint8_t funccrankTerm(uint8_t temp)
{
    return(100);
}

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
	
    if(scenario.Lambda>lambdaVoltRichTheshould)
    {
        scenario.LambDir=RICH;
    }
    else if(scenario.Lambda<lambdaVoltLeanTheshould)
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

        default:    break;
    }
		
		crtlStateOld=scenario.LambDir;
}

uint8_t funcVoleff(uint8_t PMap,uint16_t Engine_Speed)
{
    return (100);
}

uint8_t funcLambda(uint8_t PMap,uint16_t Engine_Speed)
{
    return (100);
}

uint8_t funcCycles(uint8_t temp)
{
    return (100);
}

void Injector_CMD(uint16_t pwm)
{	
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm);
}	

uint32_t Saturation(uint32_t var,uint32_t sat)
{	
		if(var>sat)
		{	
				var=sat;
		}	

    return(var);		
}

void FuelCalc(void)
{
	  if(scenario.Engine_State<CRANK)
		{	
				scenario.PW_us=0;
		}	
		else if(scenario.Engine_State==CRANK)
		{	
				//Fuel strategy
        if(scenario.TPS>maxPedalOnCrank)
        {
            scenario.PW_us=0;
        }
        else
        {
            //After achieve Engine.State=Crank Enable to inject fixed fuel amount
            scenario.Airmass=((100000*scenario.PMap*scenario.Displacement)/((28705*(scenario.Tair+273))/1000))*(scenario.Voleff/100);
						scenario.TotalTerm=(scenario.warmUpTerm*scenario.fastEnrichmentTerm*scenario.crankTerm)/10000;
						scenario.Fuelmass=(100*scenario.TotalTerm*scenario.Airmass)/scenario.AFRstoich;
						scenario.Injectormassflow=(scenario.Fuelmass*scenario.Engine_Speed)/3000;		//g/20ms
						scenario.PW_percent=scenario.Injectormassflow/150;		
						scenario.PW_us=(scenario.PW_percent*InjectorMaxTime)/1000;
						scenario.PW_us=scenario.PW_us+scenario.InjectorDeadTime;
						scenario.PW_us=Saturation(scenario.PW_us,InjectorMaxTime);
						Injector_CMD(scenario.PW_us);			
        }
		}	
		else if(scenario.Engine_State>CRANK)
		{	
				if(!(scenario.counterCycles>=funcCycles(scenario.EngineTemp)))
        {
            scenario.warmUpTerm=funcwarmUp(scenario.EngineTemp);
        }
        else
        {
            scenario.warmUpTerm=100;
        }
				
				TPS_Treatment();
				
				if(scenario.Engine_State!=OVERSPEED)
				{
						scenario.overspeedTerm=1;
				}
				else
				{
						scenario.overspeedTerm=0;
					  scenario.PW_us=0;
						Injector_CMD(scenario.PW_us);
					  return;						
				}

        scenario.Voleff=funcVoleff(scenario.PMap,scenario.Engine_Speed);
        scenario.LambdaRequested=funcLambda(scenario.PMap,scenario.Engine_Speed);
				LambdaCorrectionFunc(scenario.LambdaRequested,scenario.Lambda);			
				
				scenario.PMap=101;
				scenario.Displacement=135;
				scenario.Tair=45;
				scenario.Voleff=100;
				scenario.warmUpTerm=100;
				scenario.fastEnrichmentTerm=100;
				scenario.crankTerm=100;
				scenario.lambdaCorrectTerm=100;
				scenario.AFRstoich=132;
				scenario.Engine_Speed=3000;
				
				scenario.Airmass=((100000*scenario.PMap*scenario.Displacement)/((28705*(scenario.Tair+273))/1000))*(scenario.Voleff/100);
				scenario.TotalTerm=(scenario.warmUpTerm*scenario.fastEnrichmentTerm*scenario.lambdaCorrectTerm)/10000;
				scenario.Fuelmass=(100*scenario.TotalTerm*scenario.Airmass)/scenario.AFRstoich;
				scenario.Fuelmass=(scenario.Fuelmass*scenario.cuttOffTerm*scenario.overspeedTerm)/100;		//due overspeedTerm
				scenario.Injectormassflow=(scenario.Fuelmass*scenario.Engine_Speed)/3000;		//g/20ms
				scenario.PW_percent=scenario.Injectormassflow/150;		
				scenario.PW_us=(scenario.PW_percent*InjectorMaxTime)/1000;
				scenario.PW_us=scenario.PW_us+scenario.InjectorDeadTime;
				scenario.PW_us=Saturation(scenario.PW_us,InjectorMaxTime);
				Injector_CMD(scenario.PW_us);			
		}			
		/*
		scenario.PMap=101;
		scenario.Displacement=135;
		scenario.Tair=45;
		scenario.Voleff=100;
		scenario.warmUpTerm=100;
		scenario.fastEnrichmentTerm=100;
		scenario.crankTerm=100;
		scenario.lambdaCorrectTerm=100;
		scenario.AFRstoich=132;
		scenario.Engine_Speed=3000;
				
		scenario.Airmass=((100000*scenario.PMap*scenario.Displacement)/((28705*(scenario.Tair+273))/1000))*(scenario.Voleff/100);
		scenario.TotalTerm=(scenario.warmUpTerm*scenario.fastEnrichmentTerm*scenario.crankTerm*scenario.lambdaCorrectTerm)/1000000;
		scenario.Fuelmass=(scenario.TotalTerm*scenario.Airmass)/scenario.AFRstoich;
		//scenario.Fuelmass=(100*scenario.Airmass)/scenario.AFRstoich;
		//scenario.Fuelmass=(scenario.Fuelmass*scenario.warmUpTerm*scenario.fastEnrichmentTerm)/10000;
		//scenario.Fuelmass=(scenario.Fuelmass*scenario.crankTerm*scenario.lambdaCorrectTerm)/10000;				
		scenario.Fuelmass=(scenario.Fuelmass*scenario.cuttOffTerm*scenario.overspeedTerm)/100;		//due overspeedTerm
		scenario.Injectormassflow=(scenario.Fuelmass*scenario.Engine_Speed)/3000;		//g/20ms
		scenario.PW_percent=scenario.Injectormassflow/150;		
		scenario.PW_us=(scenario.PW_percent*InjectorMaxTime)/1000;
		scenario.PW_us=scenario.PW_us+scenario.InjectorDeadTime;
		scenario.PW_us=Saturation(scenario.PW_us,InjectorMaxTime);
		Injector_CMD(scenario.PW_us);						
		*/
}

//Will running periodically with period equal 50ms (20 times per second)
//This state machine define the engine state
void Eng_Status(void)
{
		switch(scenario.Engine_State)
		{
				case WAKEUP:    ////Pump turn on
												if(Read_Output_Pump()==OFF)
												{
														Set_Ouput_Pump(ON);
														Cond0=TRUE;
												}

												//Wait some time to fill fuel rail
												if(Timeout_ms(Cond0,&Counter0,2000))
												{
														Cond0=FALSE;													  
														Set_Ouput_Injector(ON);
													  Injector_CMD(PWM_100);
														//setTimeoutHookUp(Timer0,PrimerPulse(scenario.EngineTemp),&TurnOffInjector);
													  setTimeoutHookUp(Timer0,PrimerPulse(90),&TurnOffInjector);
														scenario.Engine_State=PRIMERINJ;
												}

												break;

				case PRIMERINJ: if(checkTimeoutHookUp(Timer0))
												{
														scenario.Engine_Speed=0;
														Set_Output_LED_Green(ON);   //Crank allowed
														scenario.Engine_State=STOP;
												}

												break;

				case STOP:      //I need to treat this statment
												if(pulseDetected)
												{
														Cond2=TRUE;   //Enable timer, timer limit to launch engine
														scenario.Engine_State=CRANK;
												}

												break;

				case CRANK:     if(scenario.Engine_Speed>idle_min)
												{
														//Enable the counter
														Cond3=TRUE;

														//Check if timer is elapsed
														if(Timeout_ms(Cond3,&Counter3,3000))
														{
																Cond3=FALSE;
															  Counter3=0;
																Set_Output_LED_Green(OFF);
																scenario.Engine_State=IDLE;
														}
												}
												else if((scenario.Acceleration==DECEL)||(Timeout_ms(Cond2,&Counter2,2000)))
												{
														Cond2=FALSE;
													  Counter2=0;
														Cond4=TRUE;
														scenario.Engine_State=STALL;
												}

												break;

				case STALL:  		if((scenario.Engine_Speed<rpm_stopped)||(Timeout_ms(Cond4,&Counter4,1500)))
												{
														Cond4=FALSE;
													  Counter4=0;
														scenario.EngineDiedCounter++;
														scenario.Engine_Speed=0;
														Set_Output_LED_Green(ON);   //Crank allowed
														scenario.Engine_State=STOP;
												}

												break;

				case IDLE:      if(scenario.Engine_Speed>idle_max)
												{
														scenario.Engine_State=CRUISE;
												}
												else if(scenario.Engine_Speed<=idle_min)
												{
														scenario.Engine_State=STALL;
													  Cond4=TRUE;
												}

												break;

				case CRUISE:    if(scenario.Engine_Speed>rpm_max)
												{
														scenario.Engine_State=OVERSPEED;
												}
												else if(scenario.Engine_Speed<=idle_max)
												{
														scenario.Engine_State=IDLE;
												}

												break;

				case OVERSPEED: if(scenario.Engine_Speed<=(rpm_max-hyst))
												{
														scenario.Engine_State=CRUISE;
												}

												break;

				default:        break;
		}
}

void Set_Pulse_Program(void)
{
    Set_Ouput_InterruptionTest();

    scenario.Measured_Period += scenario.nOverflow_RE*TMR2_16bits;

    //Engine speed must be greater than 100rpm and less than 15000rpm to consider Measured_Period useful for calculations
    if((scenario.Measured_Period<=EngineSpeedPeriod_Min)&&
       (scenario.Measured_Period>=EngineSpeedPeriod_Max))
    {
        scenario.Engine_Speed = RPM_const/scenario.Measured_Period;
        scenario.Engine_Speed_old = scenario.Engine_Speed;
        scenario.deltaEngineSpeed = scenario.Engine_Speed-scenario.Engine_Speed_old;

        //Linear prediction
        if((scenario.Engine_Speed<<1u)>scenario.Engine_Speed_old)
        {
            scenario.engineSpeedPred = (scenario.Engine_Speed<<1u)-scenario.Engine_Speed_old;
            //scenario.tdutyInputSignalPredLinear = (RPM_const*calibFlashBlock.Calibration_RAM.sensorAngDisplecement)/(scenario.engineSpeedPred*360u);
        }
        else
        {
            scenario.engineSpeedPred = 0u;
        }

        //For calculus purpose I decided to use the linear prediction
        scenario.Engine_Speed = scenario.engineSpeedPred;

        scenario.TDuty_Input_Signal += scenario.nOverflow_FE*TMR2_16bits;
    }
    else if(scenario.Measured_Period<EngineSpeedPeriod_Max)
    {
        //set flag Overspeed
        //register the max engine speed
    }
    else
    {
        //Underspeed
        //maybe I don´t need to register this because will happening all engine start event
    }

    Set_Ouput_InterruptionTest();
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Hardware_Init();
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcArray,7);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        //Update the pulse calc scenario
        if (scenario.Update_calc == TRUE)
        {
            Set_Pulse_Program();
            scenario.Update_calc = FALSE;
        }

        //Scheduler
        Periodic_task(  20,&Task_Fast,   array_sched_var, 0);
        Periodic_task( 100,&Task_Medium, array_sched_var, 1);
        Periodic_task(1000,&Task_Slow,   array_sched_var, 2);

        //Timer Management
        TimerListManagement(timerList);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 55;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 180;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        scenario.nOverflow++;
    }
}

void Rising_Edge_Event(void)
{
    scenario.Measured_Period = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
    scenario.nOverflow_RE = scenario.nOverflow;
    __HAL_TIM_SET_COUNTER(&htim2,0u);
    scenario.nOverflow = 0u;
    scenario.Rising_Edge_Counter++;
}

void Falling_Edge_Event(void)
{
    scenario.TDuty_Input_Signal = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
    scenario.nOverflow_FE = scenario.nOverflow;

    if (scenario.Rising_Edge_Counter>=2u)
    {
        scenario.Update_calc = TRUE;        //set zero after Engine Stop was detected
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if((htim->Instance == TIM2)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        Rising_Edge_Event();
    }

    if((htim->Instance == TIM2)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        Falling_Edge_Event();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
