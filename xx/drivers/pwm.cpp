/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/14
** Modified by:
** Modified date:
** Descriptions:
** STM32F4的pwm编程依赖于timer，请参照timer注释信息
** 信号频率由TIMx_ARR 寄存器值决定，其占空比则由TIMx_CCRx 寄存器值决定。
** id		baseaddr	channel 	pin
** 0~3 		TIM2 		CH1~CH4		PA0~PA3
** 4~7 		TIM3 		CH1~CH4		PC6 PC7 PB0 PB1
** 8~11 		TIM4 		CH1~CH4
** 12~15 	TIM5 		CH1~CH4
**
** 16~17 	TIM9 		CH1~CH2
** 18~19 	TIM10 		CH1~CH2
** 20~21 	TIM11 		CH1~CH2
** 22~23 	TIM12 		CH1~CH2
** 24~25 	TIM13 		CH1~CH2
** 26~27 	TIM14 		CH1~CH2
**
** 28~31 	TIM1 		CH1~CH4
** 32~35		TIM8 		CH1~CH4
**
***********************************************************************/
#include "pwm.h"

namespace driver {

struct stm32f3_pwm_hw_table
{
	IRQn_Type 			IRQn;
	TIM_HandleTypeDef		TIM_Handle;
	TIM_OC_InitTypeDef 	TIM_OC_Init;

	GPIO_TypeDef  		*GPIOx;
	GPIO_InitTypeDef   	GPIO_Init;

	pwm 					*ppwm;
};

static struct stm32f3_pwm_hw_table pwm_hw_table[] = {
	[1] = {
		.IRQn = TIM1_TRG_COM_TIM11_IRQn,
		.TIM_Handle = {
			.Instance = TIM1,
			.Init = {
	  			.ClockDivision = 0,
				.CounterMode = TIM_COUNTERMODE_UP,
				.RepetitionCounter = 0,
			},
		},
		.TIM_OC_Init= {
			.OCMode	= TIM_OCMODE_PWM1,
			.OCPolarity   = TIM_OCPOLARITY_HIGH,
			.OCFastMode   = TIM_OCFAST_DISABLE,
			.OCNPolarity  = TIM_OCNPOLARITY_HIGH,
			.OCNIdleState = TIM_OCNIDLESTATE_RESET,
			.OCIdleState  = TIM_OCIDLESTATE_RESET,
		},
		.GPIOx = GPIOE,
		.GPIO_Init = {
			.Pin =  GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14,
			.Mode = GPIO_MODE_AF_PP,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF1_TIM1,
		},
	},

	[4] = {
		.IRQn = TIM4_IRQn,
		.TIM_Handle = {
			.Instance = TIM4,
			.Init = {
	  			.ClockDivision = 0,
				.CounterMode = TIM_COUNTERMODE_UP,
				.RepetitionCounter = 0,
			},
		},
		.TIM_OC_Init= {
			.OCMode	= TIM_OCMODE_PWM1,
			.OCPolarity   = TIM_OCPOLARITY_HIGH,
			.OCFastMode   = TIM_OCFAST_DISABLE,
			.OCNPolarity  = TIM_OCNPOLARITY_HIGH,
			.OCNIdleState = TIM_OCNIDLESTATE_RESET,
			.OCIdleState  = TIM_OCIDLESTATE_RESET,
		},
		.GPIOx = GPIOD,
		.GPIO_Init = {
			.Pin =  GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
			.Mode = GPIO_MODE_AF_PP,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF2_TIM4,
		},
	},

	[3] = {
		.IRQn = TIM3_IRQn,
		.TIM_Handle = {
			.Instance = TIM3,
			.Init = {
	  			.ClockDivision = 0,
				.CounterMode = TIM_COUNTERMODE_UP,
				.RepetitionCounter = 0,
			},
		},
		.TIM_OC_Init= {
			.OCMode	= TIM_OCMODE_PWM1,
			.OCPolarity   = TIM_OCPOLARITY_HIGH,
			.OCFastMode   = TIM_OCFAST_DISABLE,
			.OCNPolarity  = TIM_OCNPOLARITY_HIGH,
			.OCNIdleState = TIM_OCNIDLESTATE_RESET,
			.OCIdleState  = TIM_OCIDLESTATE_RESET,
		},
		/* Configure PC.06 (TIM3_Channel1), PC.07 (TIM3_Channel2), PC.08 (TIM3_Channel3),
     	PC.09 (TIM3_Channel4) in output, push-pull, alternate function mode*/
		.GPIOx = GPIOC,
		.GPIO_Init = {
			.Pin =  GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
			.Mode = GPIO_MODE_AF_PP,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF2_TIM3,
		},
	},

};

pwm::pwm(PCSTR devname, s32 devid) :
	device(devname, devid),
	_channel(0),
	_dutycycle(0)
{
	pwm_hw_table[_devid].ppwm = this;
}

pwm::~pwm(void)
{
	pwm_hw_table[_devid].ppwm = NULL;
}

s32 pwm::probe(void)
{
	/* 1- Configure the TIM peripheral */
	/* -----------------------------------------------------------------------
  	TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.

	In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	since APB1 prescaler is different from 1.
		TIM3CLK = 2 * PCLK1
		PCLK1 = HCLK / 2
		=> TIM3CLK = HCLK = SystemCoreClock

	To get TIM3 counter clock at 24 MHz, the prescaler is computed as follows:
       	Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       	Prescaler = (SystemCoreClock /24 MHz) - 1

    	To get TIM3 output clock at 36 KHz, the period (ARR)) is computed as follows:
		ARR = (TIM3 counter clock / TIM3 output clock) - 1
		= 665

	TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
	TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
	TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
	TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%

	Note:
		SystemCoreClock variable holds HCLK frequency and is defined in
	system_stm32f3xx.c file.
		Each time the core clock (HCLK) changes, user had to update SystemCoreClock
	variable value. Otherwise, any configuration based on this variable will be incorrect.
	This variable is updated in three ways:
		1) by calling CMSIS function SystemCoreClockUpdate()
		2) by calling HAL API function HAL_RCC_GetSysClockFreq()
		3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  	----------------------------------------------------------------------- */

	TIM_HandleTypeDef *htim = &pwm_hw_table[_devid].TIM_Handle;
	if(HAL_TIM_PWM_GetState(htim) != HAL_TIM_STATE_RESET)
	{
		ERR("%s: failed HAL_TIM_PWM_GetState.\n", _devname);
		goto fail1;
	}
	//void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
	switch(_devid)
    	{
	case 1:
		/*TODO TIM1CLK = SystemCoreClock*2 */
		__HAL_RCC_TIM1_CLK_ENABLE();
 		__HAL_RCC_GPIOE_CLK_ENABLE();
		break;
	case 2:
		__HAL_RCC_TIM2_CLK_ENABLE();
		break;
	case 3:
	  	/* TIMx Peripheral clock enable */
	  	__HAL_RCC_TIM3_CLK_ENABLE();
	  	/* Enable all GPIO Channels Clock requested */
	  	__HAL_RCC_GPIOC_CLK_ENABLE();
		break;
    case 4:
	  	__HAL_RCC_TIM4_CLK_ENABLE();
	  	__HAL_RCC_GPIOD_CLK_ENABLE();
		break;
	default:
		break;
    	}
	/* 2- Configure peripheral GPIO */
	/* PWM channel1/2/3/4 GPIO pin configuration  */
	GPIO_TypeDef  *GPIOx = pwm_hw_table[_devid].GPIOx;
	GPIO_InitTypeDef *GPIO_Init= &pwm_hw_table[_devid].GPIO_Init;
	HAL_GPIO_Init(GPIOx, GPIO_Init);

	_irq = pwm_hw_table[_devid].IRQn;

	/* Initialize TIMx peripheral as follows:
		+ Prescaler = (SystemCoreClock / 24000000) - 1
		+ Period = (665 - 1)
		+ ClockDivision = 0
		+ Counter direction = Up
  	*/
	/* Compute the prescaler value to have TIM3 counter clock equal to 24000000 Hz */
  	htim->Init.Prescaler	= (uint32_t)(SystemCoreClock / 24000000) - 1; //24000000 Hz
  	htim->Init.Period		= PERIOD_VALUE;
	if (HAL_TIM_PWM_Init(htim) != HAL_OK)
  	{
		ERR("%s: failed HAL_TIM_PWM_Init.\n", _devname);
		goto fail1;
  	}
	_handle = (u32)htim;

	TIM_OC_InitTypeDef *pTIM_OC_Init = &pwm_hw_table[_devid].TIM_OC_Init;
	/* Set the pulse value for channel */
	if (HAL_TIM_PWM_ConfigChannel(htim, pTIM_OC_Init, TIM_CHANNEL_1) != HAL_OK) {
		ERR("%s: failed HAL_TIM_PWM_ConfigChannel-1.\n", _devname);
		goto fail0;
	}
	if (HAL_TIM_PWM_ConfigChannel(htim, pTIM_OC_Init, TIM_CHANNEL_2) != HAL_OK) {
		ERR("%s: failed HAL_TIM_PWM_ConfigChannel-2.\n", _devname);
		goto fail0;
	}
	if (HAL_TIM_PWM_ConfigChannel(htim, pTIM_OC_Init, TIM_CHANNEL_3) != HAL_OK) {
		ERR("%s: failed HAL_TIM_PWM_ConfigChannel-3.\n", _devname);
		goto fail0;
	}
	if (HAL_TIM_PWM_ConfigChannel(htim, pTIM_OC_Init, TIM_CHANNEL_4) != HAL_OK) {
		ERR("%s: failed HAL_TIM_PWM_ConfigChannel-4.\n", _devname);
		goto fail0;
	}

	INF("%s: probe success.\n", _devname);
	return 0;

fail1:
	HAL_GPIO_DeInit(GPIOx, GPIO_Init->Pin);
fail0:
	return -1;
}

s32 pwm::remove(void)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef*)_handle;
	if(HAL_TIM_Base_DeInit(htim) != HAL_OK) {
        goto fail0;
	}
	_handle = NULL;

	GPIO_TypeDef  *GPIOx = pwm_hw_table[_devid].GPIOx;
	GPIO_InitTypeDef *GPIO_Init= &pwm_hw_table[_devid].GPIO_Init;
	HAL_GPIO_DeInit(GPIOx, GPIO_Init->Pin);

	return 0;

fail0:
    return -1;
}

#if 1
s32 pwm::set_dutycycle(u32 channel, u32 dutycycle)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)_handle;
	//sConfig.Pulse = PULSE1_VALUE;
	TIM_TypeDef *TIMx = pwm_hw_table[_devid].TIM_Handle.Instance;

	if (dutycycle > 100) {
		dutycycle = 100;
	}

	dutycycle = (u32)(htim->Init.Period * dutycycle / 100);
	switch (channel)
	{
	case PWM_CHANNEL_1:
		TIMx->CCR1 = dutycycle;
		break;
	case PWM_CHANNEL_2:
		TIMx->CCR2 = dutycycle;
		break;
	case PWM_CHANNEL_3:
		TIMx->CCR3 = dutycycle;
		break;
	case PWM_CHANNEL_4:
		TIMx->CCR4 = dutycycle;
		break;
	default:
		break;
	}

	return 0;
}

s32 pwm::start(u32 channel)
{
  	/* 3- Start PWM signals generation */
  	/* Start channel 1 */
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)_handle;
	switch (channel)
	{
	case PWM_CHANNEL_1:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
		break;
	case PWM_CHANNEL_2:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
		break;
	case PWM_CHANNEL_3:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
		break;
	case PWM_CHANNEL_4:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
		break;
	default:
		break;
	}

	return 0;
}
#endif
}









/***********************************************************************
** End of file
***********************************************************************/


