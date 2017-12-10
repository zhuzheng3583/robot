/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "device.h"
#include "dma.h"
#include "semaphore.h"

namespace driver {

struct stm32_uart_hw_table
{
	GPIO_TypeDef  		*GPIOx;
	IRQn_Type 			IRQn;
	UART_HandleTypeDef	UART_Handle;
	GPIO_InitTypeDef  	GPIO_Init;

	s32                 dma_tx_id;
	s32             	dma_rx_id;
};

static struct stm32_uart_hw_table uart_hw_table[] = {
	[0] = { NULL },
	[1] = {
		.GPIOx = GPIOB,
		.IRQn = USART1_IRQn,
		.UART_Handle = {
			.Instance = USART1,
			.Init = {
				.BaudRate   = 9600,
				.WordLength = UART_WORDLENGTH_8B,
				.StopBits   = UART_STOPBITS_1,
				.Parity     = UART_PARITY_NONE,
				.HwFlowCtl  = UART_HWCONTROL_NONE,
				.Mode       = UART_MODE_TX_RX,
			},
		},
		.GPIO_Init = {
			.Pin       = GPIO_PIN_6 | GPIO_PIN_7,
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = GPIO_NOPULL,
			.Speed     = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF7_USART1,
		},
		.dma_tx_id = 124,
		.dma_rx_id = 108,
	},
	[2] = {
		.GPIOx = GPIOA,
		.IRQn = USART2_IRQn,
		.UART_Handle = {
			.Instance = USART2,
			.Init = {
				.BaudRate   = 115200,
				.WordLength = UART_WORDLENGTH_8B,
				.StopBits   = UART_STOPBITS_1,
				.Parity     = UART_PARITY_NONE,
				.HwFlowCtl  = UART_HWCONTROL_NONE,
				.Mode       = UART_MODE_TX_RX,
			},
		},
		.GPIO_Init = {
			.Pin       = GPIO_PIN_2 | GPIO_PIN_3,
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = GPIO_NOPULL,
			.Speed     = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF7_USART2,
		},
		.dma_tx_id = 52,
		.dma_rx_id = 44,
	},
	[3] = {
		.GPIOx = GPIOD,//GPIOC,
		.IRQn = USART3_IRQn,
		.UART_Handle = {
			.Instance = USART3,
			.Init = {
				.BaudRate   = 115200,
				.WordLength = UART_WORDLENGTH_8B,
				.StopBits   = UART_STOPBITS_1,
				.Parity     = UART_PARITY_NONE,
				.HwFlowCtl  = UART_HWCONTROL_NONE,
				.Mode       = UART_MODE_TX_RX,
			},
		},
		.GPIO_Init = {
			.Pin       = GPIO_PIN_8 | GPIO_PIN_9,//GPIO_PIN_10 | GPIO_PIN_11,
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = GPIO_NOPULL,
			.Speed     = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF7_USART3,
		},
		.dma_tx_id = 28,
		.dma_rx_id = 12,
	},
    
    //SBUS:串口配置为波特率100kbps，8位数据，偶校验(even)，2位停止位，无流控。
    [6] = {
		.GPIOx = GPIOC,//GPIOC,
		.IRQn = USART6_IRQn,
		.UART_Handle = {
			.Instance = USART6,
			.Init = {
				.BaudRate   = 100000,
				.WordLength = UART_WORDLENGTH_8B,
				.StopBits   = UART_STOPBITS_2,
				.Parity     = UART_PARITY_EVEN,
				.HwFlowCtl  = UART_HWCONTROL_NONE,
				.Mode       = UART_MODE_RX,
			},
		},
		.GPIO_Init = {
			.Pin       = GPIO_PIN_7,
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = GPIO_NOPULL,
			.Speed     = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF8_USART6,
		},
		.dma_tx_id = -1,
		.dma_rx_id = 77,
	},
    
};
  
class uart : public device
{
public:
    uart(PCSTR devname, s32 devid);
    ~uart(void);
    
public:
    static uart *owner[ARRAYSIZE(uart_hw_table)];
    
public:
    u32 _baudrate;

    dma *_dmatx;
    dma *_dmarx;
    s32 _dma_tx_id;
    s32 _dma_rx_id;

    s32 _flag_tx;
    s32 _flag_rx;

    semaphore *_sem_tx;
    semaphore *_sem_rx;
    
public:
    virtual s32 probe(void);
    virtual s32 remove(void);

    inline s32 recv(u8 *buf, u32 count);
    inline s32 send(u8 *buf, u32 count);

    void set_baudrate(u32 baudrate)	{ _baudrate = baudrate; }
    u32 get_baudrate(void)			{ return _baudrate; }

    virtual s32 self_test(void);

public:
    virtual s32 read(u8 *buf, u32 size);
    virtual s32 write(u8 *buf, u32 size);

public:    
    virtual void isr(void);

};

}

/***********************************************************************
** End of file
***********************************************************************/
