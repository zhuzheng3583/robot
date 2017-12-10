/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/05
** Modified by:
** Modified date:
** Descriptions:
**
** STM32F4的定时器分为以下3大类:
** 1.基本定时器:TIM6和TIM7,无PWM输出捕捉功能
** 2.通用定时器:TIM2到TIM5,包含基本定时所有功能同时产生多达4路的PWM输出
** 3.通用定时器:TIM9到TIM14,包含基本定时所有功能同时产生多达2路的PWM输出
** 4.高级定时器:TIM1和TIM8,包含通用定时器所有功能同时带可编程死区的互补输出。
**
** id baseaddr 	modulefreq 	irq
** 0  NULL   		NULL			NULL
** 1  TIM1  		0			TIM1_CC_IRQn
** 2  TIM2		0			TIM2_IRQn
** 3  TIM3		0			TIM3_IRQn
** 4  TIM4		0			TIM4_IRQn
** 5  TIM5		0			TIM5_IRQn
** 6  TIM6		0			TIM6_DAC_IRQn
** 7  TIM7		0			TIM7_IRQn
** 8  TIM8		0			TIM8_CC_IRQn
** 9  TIM9		0			TIM1_BRK_TIM9_IRQn
** 10 TIM10 		0			TIM1_UP_TIM10_IRQn
** 11 TIM11 		0			TIM1_TRG_COM_TIM11_IRQn
** 12 TIM12 		0			TIM8_BRK_TIM12_IRQn
** 13 TIM13 		0			TIM8_UP_TIM13_IRQn
** 14 TIM14 		0			TIM8_TRG_COM_TIM14_IRQn
**
***********************************************************************/
#include "timer.h"


namespace driver {

struct stm32f3_timer_hw_table
{
	IRQn_Type 			IRQn;
	TIM_HandleTypeDef		TIM_Handle;
	
	timer 				*ptimer;
};

static struct stm32f3_timer_hw_table timer_hw_table[] = {
	[2] = { 
		.IRQn = TIM2_IRQn,
		.TIM_Handle = {
			.Instance = TIM2,
			.Init = {
	  			.ClockDivision = 0,
				.CounterMode = TIM_COUNTERMODE_UP,
			},
		},
	},
	[3] = { 
		.IRQn = TIM3_IRQn,
		.TIM_Handle = {
			.Instance = TIM3,
			.Init = {
	  			.ClockDivision = 0,
				.CounterMode = TIM_COUNTERMODE_UP,
			},
		},
	},
};

void timer_func_test(void *arg)
{
	WRN("timer_func_test: Called to timer_func_test, arg = %d. Please set_function...\n", (u32)arg);
}

//id = [1,17]
timer::timer(PCSTR name, s32 id) :
	device(name, id),
	_func(NULL),
	_func_arg(NULL)
{
	timer_hw_table[_id].ptimer = this;
}

timer::~timer(void)
{
	timer_hw_table[_id].ptimer = NULL;
}

s32 timer::probe(void)
{
  	/* 1- Configure the TIM peripheral */ 
  	/* -----------------------------------------------------------------------
	In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
	since APB1 prescaler is different from 1.   
		TIM3CLK = 2 * PCLK1  
		PCLK1 = HCLK / 2 
		=> TIM3CLK = HCLK = SystemCoreClock
	To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
	Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	Prescaler = (SystemCoreClock /10 KHz) - 1
       
	Note: 
		SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
	variable value. Otherwise, any configuration based on this variable will be incorrect.
	This variable is updated in three ways:
		1) by calling CMSIS function SystemCoreClockUpdate()
		2) by calling HAL API function HAL_RCC_GetSysClockFreq()
		3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}

	TIM_HandleTypeDef *htim = &timer_hw_table[_id].TIM_Handle;
	if(HAL_TIM_Base_GetState(htim) != HAL_TIM_STATE_RESET)
	{
		ERR("%s: failed HAL_TIM_Base_GetState.\n", _name);
		goto fail1;	
	}
	//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
	switch(_id)
    	{
	case 1:
		__HAL_RCC_TIM1_CLK_ENABLE();
		break;
	case 2:
		__HAL_RCC_TIM2_CLK_ENABLE();
		break;
	case 3:
		 /* TIMx Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();
		break;
	default:
		break;
    	}

	_irq = timer_hw_table[_id].IRQn; 
	_handle = (u32)htim;

  	/* Initialize TIM3 peripheral as follows:
		+ Period = 10000 - 1
		+ Prescaler = (SystemCoreClock/10000) - 1
		+ ClockDivision = 0
		+ Counter direction = Up
     */
	timer::set_timeout(1000);
	
	/* Configure the NVIC for TIMx */
	device::request_irq(_irq, this);
	device::enable_irq(_irq); 	
	
	timer::set_function(timer_func_test, (void *)1234);
	INF("%s: probe success.\n", _name);
	return 0;

fail1:
	device::remove();
fail0:
	return -1;	
}

s32 timer::remove(void)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef*)_handle;
		
	if (device::remove() < 0) {
		goto fail0;
	}
		
	if(HAL_TIM_Base_DeInit(htim) != HAL_OK) {
    		goto fail1;
	}

	_handle = NULL;

	return 0;

fail1:
fail0:
    return -1;	
}

s32 timer::set_timeout(u32 ms)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef*)_handle;
	htim->Init.Period = ms * 10 - 1;//10000 - 1(1S)
	/* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	htim->Init.Prescaler = (SystemCoreClock / 10000) - 1, //10KHz
	htim->Init.ClockDivision = 0,
	htim->Init.CounterMode = TIM_COUNTERMODE_UP,
	DBG("%s: timeout = %dms, Period = %d, Prescaler = %d.\n", 
		_name, ms, htim->Init.Period, htim->Init.Prescaler);
	if(HAL_TIM_Base_Init(htim) != HAL_OK)
	{
		ERR("%s: failed to set_timeout.\n", _name);
		return -1;
	}

	return 0;
}


s32 timer::set_function(timer_func_t func, void *arg)
{
	_func = func;
	_func_arg = arg;

	return 0;
}

s32 timer::start(void)
{
	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
	/* Start Channel1 */
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef*)_handle;
  	if(HAL_TIM_Base_Start_IT(htim) != HAL_OK) {
		ERR("%s: failed to start.\n", _name);
		return -1;
  	}

	INF("%s: start success.\n", _name);
	return 0;
}

s32 timer::stop(void)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef*)_handle;
  	if(HAL_TIM_Base_Stop_IT(htim) != HAL_OK) {
		ERR("%s: failed to stop.\n", _name);
		return -1;
  	}

	INF("%s: stop success.\n", _name);
	return 0;
}



s32 timer::self_test(void)
{
	timer::set_timeout(500);
	timer::set_function(timer_func_test, (void *)1234);
	timer::start();
	return 0;
}

void timer::isr(void)
{
	HAL_TIM_IRQHandler((TIM_HandleTypeDef*)_handle);
}

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timer *ptimer = NULL;
	for (s32 id = 0; id < ARRAYSIZE(timer_hw_table); id++) {
		if (timer_hw_table[id].ptimer->_id == id) {
			ptimer = timer_hw_table[id].ptimer;
		}
	}

	if (ptimer != NULL) {
		ptimer->_func(ptimer->_func_arg);
	}
}

#ifdef __cplusplus
}
#endif

}
/***********************************************************************
** End of file
***********************************************************************/

