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
#include "uart.h"
#include "core.h"

#define UART_POLLING_TIMEOUT_MS 	1000
#define UART_IT_TIMEOUT_MS			10
#define UART_DMA_TIMEOUT_MS 		5000

#define UART_POLLING_MODE			(1 << 0)
#define UART_IT_MODE				(1 << 1)
#define UART_DMA_MODE				(1 << 2)
#define UART_MODE 				    UART_DMA_MODE


namespace driver {
  
uart *uart::owner[ARRAYSIZE(uart_hw_table)] = { NULL };


uart::uart(PCSTR name, s32 id) :
	device(name, id),
	_baudrate(115200),
	_flag_tx(0),
	_flag_rx(0)
{
	// For stm32_cube_lib C callback
    uart::owner[_id] = this;
    
    _sem_tx = new semaphore;
    _sem_rx = new semaphore;
}

uart::~uart(void)
{
    uart::owner[_id] = NULL;
    delete _sem_tx;
    delete _sem_rx;
}

s32 uart::probe(void)
{
    _sem_tx->create("uart_sem_tx", 1);
    _sem_rx->create("uart_sem_rx", 1);
    
	UART_HandleTypeDef *huart = &uart_hw_table[_id].UART_Handle;
	if(HAL_UART_GetState(huart) != HAL_UART_STATE_RESET)
	{
		//ERR("%s: failed HAL_UART_GetState.\n", _name);
		goto fail0;
	}

	//void HAL_UART_MspInit(UART_HandleTypeDef *huart)
    	switch(_id)
    	{
	case 1:
		/* 1- Enable USARTx peripherals and GPIO Clocks */
		//__HAL_RCC_GPIOE_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_USART1_CLK_ENABLE();
		break;
	case 2:
		__HAL_RCC_GPIOA_CLK_ENABLE();
  		__HAL_RCC_USART2_CLK_ENABLE();
		break;
	case 3:
        __HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
  		__HAL_RCC_USART3_CLK_ENABLE();
		break; 
	case 6:
		__HAL_RCC_GPIOC_CLK_ENABLE();
  		__HAL_RCC_USART6_CLK_ENABLE();
		break;
	default:
        CAPTURE_ERR();
		break;
    	}
	/* 2- Configure peripheral GPIO */
	/* UART TX/RX GPIO pin configuration  */
	GPIO_TypeDef  *GPIOx = uart_hw_table[_id].GPIOx;
	GPIO_InitTypeDef *GPIO_Init= &uart_hw_table[_id].GPIO_Init;
	HAL_GPIO_Init(GPIOx, GPIO_Init);

	_irq = uart_hw_table[_id].IRQn;
	_dma_tx_id = uart_hw_table[_id].dma_tx_id;
	_dma_rx_id = uart_hw_table[_id].dma_rx_id;

	/* 3- Configure the UART peripheral*/
  	if(HAL_UART_Init(huart) != HAL_OK) {
		goto fail1;
  	}
	_handle = (u32)huart;

#if (UART_MODE == UART_DMA_MODE)
    u8 str[16] = {0};
    if (_dma_tx_id > 0) {
        snprintf((char *)str, 16, "dma-%d", _dma_tx_id);
        _dmatx = new dma((PCSTR)str, _dma_tx_id);
        //INF("%s: new dmatx %s[dma%d,stream%d,channel%d].\n", _name, str, _dmatx->_dma_id, _dmatx->_stream_id, _dmatx->_channel_id);
        _dmatx->probe();
        _dmatx->config(DMA_DIR_MEM_TO_PERIPH, DMA_ALIGN_BYTE, DMA_PRI_LOW, 1, 0);
        /* Associate the initialized DMA handle to the UART handle */
        __HAL_LINKDMA(huart, hdmatx, *(DMA_HandleTypeDef *)(_dmatx->_handle));
    }
    if (_dma_rx_id > 0) {
        snprintf((char *)str, 16, "dma-%d", _dma_rx_id);
        _dmarx = new dma((PCSTR)str, _dma_rx_id);
        //INF("%s: new dmarx %s[dma%d,stream%d,channel%d].\n", _name, str, _dmarx->_dma_id, _dmarx->_stream_id, _dmarx->_channel_id);
        _dmarx->probe();
        _dmarx->config(DMA_DIR_PERIPH_TO_MEM, DMA_ALIGN_BYTE, DMA_PRI_HIGH, 0, 1);
        __HAL_LINKDMA(huart, hdmarx, *(DMA_HandleTypeDef *)(_dmarx->_handle));
    }
#endif
#if (UART_MODE == UART_IT_MODE || UART_MODE == UART_DMA_MODE)
	/*##-3- Configure the NVIC for UART ########################################*/
	/* NVIC for USART */
	device::request_irq(_irq, this);
	device::enable_irq(_irq);
#endif

	//INF("%s: probe success.\n", _name);
	return 0;

fail1:
	HAL_GPIO_DeInit(GPIOx, GPIO_Init->Pin);
fail0:
	return -1;
}

s32 uart::remove(void)
{
	UART_HandleTypeDef *huart = (UART_HandleTypeDef *)_handle;
#if (UART_MODE == UART_IT_MODE || UART_MODE == UART_DMA_MODE)
	device::disable_irq(_irq);
#endif
#if (UART_MODE == UART_DMA_MODE)
	_dmatx->remove();
	_dmarx->remove();
	delete _dmatx;
	delete _dmarx;
	_dmatx = NULL;
	_dmarx = NULL;
#endif
	if(HAL_UART_DeInit(huart)!= HAL_OK) {
        goto fail0;
	}

	//void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
    	switch(_id)
    	{
	case 1:

		/*##-1- Reset peripherals ##################################################*/
	 	__HAL_RCC_USART1_FORCE_RESET();
	 	__HAL_RCC_USART1_RELEASE_RESET();
		break;
	case 2:
		__HAL_RCC_USART2_FORCE_RESET();
	 	__HAL_RCC_USART2_RELEASE_RESET();
		break;
	case 3:
		__HAL_RCC_USART3_FORCE_RESET();
	 	__HAL_RCC_USART3_RELEASE_RESET();
		break;
	default:
		break;
    	}
	/*##-2- Disable peripherals and GPIO Clocks #################################*/
	GPIO_TypeDef  *GPIOx = uart_hw_table[_id].GPIOx;
	uint32_t GPIO_Pin = uart_hw_table[_id].GPIO_Init.Pin;
	HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

	_handle = NULL;
	return 0;

fail0:
    return -1;
}

s32 uart::recv(u8 *buf, u32 count)
{
    if (!(count > 0)) {
        return 0;
    }
    
    u32 readcnt = 0;

#if (UART_MODE == UART_POLLING_MODE)
	if(HAL_UART_Receive((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count, UART_POLLING_TIMEOUT_MS) != HAL_OK) {
    		//CAPTURE_ERR();
	}
    readcnt = count;
#elif (UART_MODE == UART_IT_MODE)
	if(HAL_UART_Receive_IT((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
	//pend _flag_rx
	s32 ret = 0;
	wait_condition_ms(_flag_rx == 1, UART_IT_TIMEOUT_MS, &ret);
	if (ret < 0) {
		return -1;
	} else {
		_flag_rx = 0;
	}
    readcnt = count;
#elif (UART_MODE == UART_DMA_MODE)
	if(HAL_UART_Receive_DMA((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
#if 1
    if (_sem_rx->pend(UART_DMA_TIMEOUT_MS) == true) {
        readcnt = count - _dmarx->get_leftover_count();
    } else {
        readcnt = count - _dmarx->get_leftover_count();
    }
#else
	//pend _flag_rx
	s32 ret = 0;
	wait_condition_ms(_flag_rx == 1, UART_DMA_TIMEOUT_MS, &ret);
	if (ret < 0) {
		readcnt = count - _dmarx->get_leftover_count();
	} else {
		_flag_rx = 0;
		readcnt = count;
	}
#endif
#endif
	return readcnt;
}

s32 uart::send(u8 *buf, u32 count)
{
    if (!(count > 0)) {
        return 0;
    }
    
    u32 writecnt = 0;

#if (UART_MODE == UART_POLLING_MODE)
	if(HAL_UART_Transmit((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count, UART_POLLING_TIMEOUT_MS) != HAL_OK) {
        //CAPTURE_ERR();
	}
    writecnt = count;
#elif (UART_MODE == UART_IT_MODE)
	if(HAL_UART_Transmit_IT((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
	//pend tx_event
	s32 ret = 0;
	wait_condition_ms(_flag_tx == 1, UART_IT_TIMEOUT_MS, &ret);
	if (ret < 0) {
		return -1;
	} else {
		_flag_tx = 0;
	}
    writecnt = count;
#elif (UART_MODE == UART_DMA_MODE)
	if(HAL_UART_Transmit_DMA((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
#if 1
	//pend tx_event
    if (_sem_tx->pend(UART_DMA_TIMEOUT_MS) == true) {
        writecnt = count - _dmatx->get_leftover_count();
    } else {
        writecnt = count - _dmatx->get_leftover_count();
    }
#else	
    s32 ret = 0;
	wait_condition_ms(_flag_tx == 1, UART_DMA_TIMEOUT_MS, &ret);
	if (ret < 0) {
		writecnt = count - _dmatx->get_leftover_count();
	} else {
		_flag_tx = 0;
        writecnt = count;
	}
#endif
#endif
	return writecnt;
}

s32 uart::self_test(void)
{
	uart::open();
#if 1
	u8 wbuf[16] = "hello world!";
	u8 rbuf[25] = { 0 };

    uart::write(wbuf, ARRAYSIZE(wbuf));
    while (1) {
        uart::read(rbuf, sizeof(rbuf));
        for (u32 i = 0; i < sizeof(rbuf); i++) {
            DBG("rbuf[%d] = 0x%02x ", i, rbuf[i]);
        }
        //INF("%s", rbuf);

        //uart::write(rbuf, ARRAYSIZE(rbuf));
        //core::mdelay(1);
    }
#elif 0
	u32 n = 0;
	while (1) {
		INF("%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d.\n",
			n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n);
		n++;
	}
	core::mdelay(100);
#else
    u32 n = 0;
    u8 wbuf[17] = "hello world!";
	while (1) {
        uart::write(wbuf, ARRAYSIZE(wbuf));
        core::mdelay(100);
	}
	
#endif
	uart::close();
}

s32 uart::read(u8 *buf, u32 size)
{
	return recv(buf, size);
}

s32 uart::write(u8 *buf, u32 size)
{
	return send(buf, size);
}

void uart::isr(void)
{
	HAL_UART_IRQHandler((UART_HandleTypeDef *)_handle);
}

#ifdef __cplusplus
extern "C" {
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//post tx_event
	uart *puart = NULL;
	for (s32 id = 0; id < ARRAYSIZE(uart::owner); id++) {
		if (uart::owner[id]->_handle == (u32)huart) {
			puart = uart::owner[id];
            break;
		}
	}

	if (puart != NULL) {
        puart->_flag_tx = 1;
        puart->_sem_tx->post(UART_DMA_TIMEOUT_MS);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//post rx_event
    uart *puart = NULL;
	for (s32 id = 0; id < ARRAYSIZE(uart::owner); id++) {
		if (uart::owner[id]->_handle == (u32)huart) {
			puart = uart::owner[id];
            break;
		}
	}

	if (puart != NULL) {
		puart->_flag_rx = 1;
        puart->_sem_rx->post(UART_DMA_TIMEOUT_MS);
        /* notify anyone waiting for data */
        puart->poll_notify(POLLIN);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart *puart = NULL;
	for (s32 id = 0; id < ARRAYSIZE(uart::owner); id++) {
		if (uart::owner[id]->_handle == (u32)huart) {
			puart = uart::owner[id];
            break;
		}
	}

	if (puart != NULL) {
	}
}

#ifdef __cplusplus
}
#endif

}



/***********************************************************************
** End of file
***********************************************************************/

