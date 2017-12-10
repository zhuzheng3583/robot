/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/12
** Modified by:
** Modified date:
** Descriptions:
** STM32F4�����9��IO(A/B/C/D/E/F/G/H/I)��ÿ����16��IO�ܽţ���9*16=144��IO�ܽ�
** ����144��ÿ��IO�ܽų����һ���豸�����豸IDΪ����[0, 143]����������:
** A:[0, 15]
** B:[16, 31]
** C:[32, 47]
** D:[48, 63]
** E:[64, 79]
** F:[80, 95]
** G:[96, 111]
** H:[112, 127]
** I:[128, 143]
**
** 	�ⲿ�жϣ� 
	������˵���ⲿ�жϺ�3��ֻ����ӳ�䵽����PA3��PB3��PC3��PD3��PE3��PF3��PG3
��һ���е�����һ������һ�������Ƕ���������ϣ�������ӳ�����������ϡ� 
	ͬ���ⲿ�жϺ�4��ֻ����ӳ�䵽����PA4��PB4��PC4��PD4��PE4��PF4��PG4
��һ���е�����һ�������ϣ�������ӳ�����������ϡ� 

	���⣬�ⲿ�ж�ֻ��7������ӿڣ��ⲿ�жϺ�0~4
���ž��ж������жϴ���ӿڣ��ⲿ�жϺ�5~9������һ���жϴ���ӿڣ��ⲿ�жϺ�
10~15������һ���жϴ���ӿڡ� 

    ��������ԭ���ⲿ�жϾ���ֻʹ�����ź�0~4����5~9�е�ĳһ������10~15
�е�ĳһ�����Լ��жϴ���ı�д��ͬʱҲ�������ⲿ�жϵĴ���ʱ�䣨�ⲿ�жϺ�
5~9��10~15�������ֹ��Ӧһ���жϺţ�����Ҫ���жϴ��������ȷ�����ĸ����Ų������жϣ��� 
***********************************************************************/
#include "gpio.h"

namespace driver {

static GPIO_TypeDef *bank_table[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};
static IRQn_Type irq_table[] = {EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn};
static uint16_t index_table[] = { 
	GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, 
	GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, 
	GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, 
	GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_All};

gpio::gpio(PCSTR name, s32 id) : 
	device(name, id)
{

}

gpio::~gpio(void)
{
	
}

s32 gpio::probe(void)
{		
	ASSERT(_id < GPIO_PIN_NUM);
	_bank	= _id / GPIO_PIN_NUM_PER_BANK;
	_index	= _id % GPIO_PIN_NUM_PER_BANK;
	INF("%s: bank = %d, index = %d.\n", _name, _bank, _index);
	
	if (device::probe() < 0) {
		CAPTURE_ERR();  
		ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}

    	switch(_bank)
    	{
	case 0:
		__HAL_RCC_GPIOA_CLK_ENABLE();
		break;
	case 1:
		__HAL_RCC_GPIOB_CLK_ENABLE();
		break;
	case 2:
		__HAL_RCC_GPIOC_CLK_ENABLE();
		break;
	case 3:
		__HAL_RCC_GPIOD_CLK_ENABLE();
		break;
	case 4:
		__HAL_RCC_GPIOE_CLK_ENABLE();
		break;
	case 5:
		__HAL_RCC_GPIOF_CLK_ENABLE();
		break;
	default:
		break;
    	}

	INF("%s: probe success.\n", _name);
	return 0;

fail0:
	return -1;	
}

s32 gpio::remove(void)
{
	if (device::remove() < 0) {
		goto fail0;
	}

	HAL_GPIO_DeInit(bank_table[_bank], index_table[_index]);
	return 0;

fail0:
    return -1;
}

s32 gpio::set_direction_input(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin = index_table[_index];
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(bank_table[_bank], &GPIO_InitStruct);
	return 0;
}

s32 gpio::set_direction_output(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin = index_table[_index];
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Pull = GPIO_PULLUP;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(bank_table[_bank], &GPIO_InitStruct);
	return 0;
}

s32 gpio::set_gpio_to_irq(void)
{
	/* connected to (PE6) IOs in External Interrupt Mode with Rising edge trigger detection. */
	GPIO_InitTypeDef  GPIO_InitStruct;
  	GPIO_InitStruct.Pin = index_table[_index];
  	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(bank_table[_bank], &GPIO_InitStruct);
	
  	/* Enable and set EXTI0 Interrupt to the lowest priority */
	_irq = irq_table[_index];
	device::request_irq(_irq, this);
	device::enable_irq(_irq); 	
  
	return 0;
}

void gpio::set_value(u8 value)
{
	HAL_GPIO_WritePin(bank_table[_bank], index_table[_index], value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

u8 gpio::get_value(void)
{
	return ((u8)HAL_GPIO_ReadPin(bank_table[_bank], index_table[_index]));
}

s32 gpio::self_test(void)
{
	gpio::set_direction_output();
	while (1) {
		gpio::set_value(1);
		core::mdelay(500);
		gpio::set_value(0);
		core::mdelay(500);
	}
	
}

void gpio::isr(void)
{
	//HAL_GPIO_EXTI_IRQHandler(index_table[_index]);
	/* EXTI line interrupt detected */
  	if(__HAL_GPIO_EXTI_GET_IT(index_table[_index]) != RESET)
  	{
    		__HAL_GPIO_EXTI_CLEAR_IT(index_table[_index]);
    		//TODO here HAL_GPIO_EXTI_Callback
    		INF("%s: isr gpio[bank%d,index%d].\n", _name, _bank, _index);
  	}
}

}
/***********************************************************************
** End of file
***********************************************************************/

