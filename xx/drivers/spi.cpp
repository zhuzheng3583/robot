/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "spi.h"

#define SPI_POLLING_TIMEOUT_MS 		1//1000
#define SPI_IT_TIMEOUT_MS			10
#define SPI_DMA_TIMEOUT_MS 		10

#define SPI_POLLING_MODE			(1 << 0)
#define SPI_IT_MODE				(1 << 1)
#define SPI_DMA_MODE				(1 << 2)

#define SPI_MODE 					SPI_POLLING_MODE

namespace driver {

struct stm32_spi_hw_table
{
	GPIO_TypeDef  		*GPIOx;
	IRQn_Type 			IRQn;
	SPI_HandleTypeDef		SPI_Handle;
	GPIO_InitTypeDef  	GPIO_Init;

	s32             		dma_tx_id;
	s32             		dma_rx_id;
	spi 					*pspi;
};


static struct stm32_spi_hw_table spi_hw_table[] = {
	[0] = { NULL },
	[1] = {
		.GPIOx = GPIOA,
		.IRQn = SPI1_IRQn,
		.SPI_Handle = {
			.Instance = SPI1,
			.Init = {
				.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,//SPI_BAUDRATEPRESCALER_16,
				.Direction         = SPI_DIRECTION_2LINES,
				.CLKPhase          = SPI_PHASE_2EDGE,//SPI_PHASE_1EDGE,//
				.CLKPolarity       = SPI_POLARITY_HIGH,//SPI_POLARITY_LOW,//
				.CRCCalculation    = SPI_CRCCALCULATION_DISABLE,
				.CRCPolynomial     = 7,
				.DataSize          = SPI_DATASIZE_8BIT,
				.FirstBit          = SPI_FIRSTBIT_MSB,
				.NSS               = SPI_NSS_SOFT,
				.TIMode            = SPI_TIMODE_DISABLE,
				//.NSSPMode          = SPI_NSS_PULSE_DISABLE,
				//.CRCLength         = SPI_CRC_LENGTH_8BIT,
				.Mode 			 = SPI_MODE_MASTER,
			},
		},
		.GPIO_Init = {
			.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = GPIO_NOPULL,
			.Speed     = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF5_SPI1,
		},
		.dma_tx_id = 91,
		.dma_rx_id = 83,
	},
};

spi::spi(PCSTR name, s32 id) :
	device(name, id)
{
	// For stm32_cube_lib C callback
	spi_hw_table[_id].pspi = this;

}

spi::~spi(void)
{
    spi_hw_table[_id].pspi = NULL;
}

s32 spi::probe(void)
{
	SPI_HandleTypeDef *hspi = &spi_hw_table[_id].SPI_Handle;
	if(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_RESET)
	{
		ERR("%s: failed HAL_SPI_GetState.\n", _name);
		goto fail0;
	}

	//void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
    	switch(_id)
    	{
	case 1:
    		/* 1- Enable peripherals and GPIO Clocks */
    		/* Enable GPIO TX/RX clock */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_SPI1_CLK_ENABLE();
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
    	}
	/* 2- Configure peripheral GPIO */
	/* SPI SCK GPIO pin configuration  */
	GPIO_TypeDef  *GPIOx = spi_hw_table[_id].GPIOx;
	GPIO_InitTypeDef *GPIO_Init= &spi_hw_table[_id].GPIO_Init;
	HAL_GPIO_Init(GPIOx, GPIO_Init);

	_irq = spi_hw_table[_id].IRQn;
	_dma_tx_id = spi_hw_table[_id].dma_tx_id;
	_dma_rx_id = spi_hw_table[_id].dma_rx_id;

	/* 3- Configure the SPI peripheral*/
  	if(HAL_SPI_Init(hspi) != HAL_OK) {
		goto fail1;
  	}
	_handle = (u32)hspi;

#if (SPI_MODE == SPI_DMA_MODE)
	s8 str[16];
	snprintf((char *)str, 16, "dma-%d", _dma_tx_id);
	_dmatx = new dma((PCSTR)str, _dma_tx_id);
	INF("%s: new dmatx %s[dma%d,stream%d,channel%d].\n",
        _name, str, _dmatx->_dma_id, _dmatx->_channel_id);
	_dmatx->probe();
	_dmatx->config(DMA_DIR_MEM_TO_PERIPH, DMA_ALIGN_BYTE, DMA_PRI_LOW, 1, 0);
	__HAL_LINKDMA(hspi, hdmatx, *(DMA_HandleTypeDef *)(_dmatx->_handle));

    snprintf((char *)str, 16, "dma-%d", _dma_rx_id);
	_dmarx = new dma((PCSTR)str, _dma_rx_id);
	INF("%s: new dmarx %s[dma%d,stream%d,channel%d].\n",
        _name, str, _dmarx->_dma_id, _dmarx->_channel_id);
	_dmarx->probe();
	_dmarx->config(DMA_DIR_PERIPH_TO_MEM, DMA_ALIGN_BYTE, DMA_PRI_HIGH, 0, 1);
	__HAL_LINKDMA(hspi, hdmarx, *(DMA_HandleTypeDef *)(_dmarx->_handle));
#endif
#if (SPI_MODE == SPI_IT_MODE || SPI_MODE == SPI_DMA_MODE)
	/*##-3- Configure the NVIC for SPI ########################################*/
	/* NVIC for SPI */
	//interrupt::request_irq(_irq, this);
	//interrupt::enable_irq(_irq);
#endif

	INF("%s: probe success.\n", _name);
	return 0;

fail1:
	HAL_GPIO_DeInit(GPIOx, GPIO_Init->Pin);
fail0:
	return -1;
}

s32 spi::remove(void)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)_handle;

#if (SPI_MODE == SPI_IT_MODE || SPI_MODE == SPI_DMA_MODE)
	interrupt::disable_irq(_irq);
#endif
#if (SPI_MODE == SPI_DMA_MODE)
	_dmatx->remove();
	_dmarx->remove();
	delete _dmatx;
	delete _dmarx;
	_dmatx = NULL;
	_dmarx = NULL;
#endif
	if(HAL_SPI_DeInit(hspi)!= HAL_OK) {
    		goto fail0;
	}

	//void HAL_SPI_MspDeInit(UART_HandleTypeDef *huart)
    	switch(_id)
    	{
	case 1:
		/*##-1- Reset peripherals */
	 	__HAL_RCC_SPI1_FORCE_RESET();
	 	__HAL_RCC_SPI1_RELEASE_RESET();
		break;
	case 2:
		__HAL_RCC_SPI2_FORCE_RESET();
	 	__HAL_RCC_SPI2_RELEASE_RESET();
		break;
	case 3:
		__HAL_RCC_SPI3_FORCE_RESET();
	 	__HAL_RCC_SPI3_RELEASE_RESET();
		break;
	default:
		break;
    	}
	/*##-2- Disable peripherals and GPIO Clocks */
	GPIO_TypeDef  *GPIOx = spi_hw_table[_id].GPIOx;
	uint32_t GPIO_Pin = spi_hw_table[_id].GPIO_Init.Pin;
	HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

	_handle = NULL;
	return 0;

fail0:
    return -1;
}

s32 spi::tx(u8 *buf, u32 size)
{
	HAL_StatusTypeDef status = HAL_OK;
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)_handle;

	u16 len = (u16)size;
#if (SPI_MODE == SPI_POLLING_MODE)
	status = HAL_SPI_Transmit(hspi, buf, len, SPI_POLLING_TIMEOUT_MS);
#elif (SPI_MODE == SPI_DMA_MODE)
	status = HAL_SPI_Transmit_DMA(hspi, buf, len);
    //pend eventtxrx
    s32 ret = 0;
    wait_condition_ms(_eventtxrx == 1, SPI_DMA_TIMEOUT_MS, &ret);
    if (ret < 0) {
        return -1;
    } else {
        _eventtxrx = 0;
    }

#elif (SPI_MODE == SPI_IT_MODE)
#error "No SPI interrupt mode!"
#endif
	if(status != HAL_OK) {
		ERR("%s: failed to spi tx!\n", _name);
		return -1;
	}

	//TODO 添加mutex
	//mutex_unlock(&d->i2c_mutex);
	return 0;
}

s32 spi::rx(u8 *buf, u32 size)
{
	HAL_StatusTypeDef status = HAL_OK;
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)_handle;

	u16 len = (u16)size;
#if (SPI_MODE == SPI_POLLING_MODE)
	status = HAL_SPI_Receive(hspi, buf, len, SPI_POLLING_TIMEOUT_MS);
#elif (SPI_MODE == SPI_DMA_MODE)
	status = HAL_SPI_Receive_DMA(hspi, buf, len);
    //pend eventtxrx
    s32 ret = 0;
    wait_condition_ms(_eventtxrx == 1, SPI_DMA_TIMEOUT_MS, &ret);
    if (ret < 0) {
        return -1;
    } else {
        _eventtxrx = 0;
    }

#elif (SPI_MODE == SPI_IT_MODE)
#error "No SPI interrupt mode!"
#endif
	if(status != HAL_OK) {
		ERR("%s: failed to spi rx!\n", _name);
		return -1;
	}

	//TODO 添加mutex
	//mutex_unlock(&d->i2c_mutex);
	return 0;
}

s32 spi::transfer(struct spi_msg *msg)
{
	HAL_StatusTypeDef status = HAL_OK;
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)_handle;

	//TODO 添加mutex
	//if (mutex_lock_interruptible(&d->i2c_mutex) < 0)
	//	return -EAGAIN;

	u16 len = msg->len;
	u8 *buf = (u8 *)(msg->buf);
	if (msg->flags & SPI_M_RD) {
#if (SPI_MODE == SPI_POLLING_MODE)
		status = HAL_SPI_Receive(hspi, buf, len, SPI_POLLING_TIMEOUT_MS);
	} else {
		status = HAL_SPI_Transmit(hspi, buf, len, SPI_POLLING_TIMEOUT_MS);
	}
#elif (SPI_MODE == SPI_DMA_MODE)
		status = HAL_SPI_Receive_DMA(hspi, buf, len);
	} else {
		status = HAL_SPI_Transmit_DMA(hspi, buf, len);
	}
    //pend eventtxrx
    s32 ret = 0;
    wait_condition_ms(_eventtxrx == 1, SPI_DMA_TIMEOUT_MS, &ret);
    if (ret < 0) {
        return -1;
    } else {
        _eventtxrx = 0;
    }
#elif (SPI_MODE == SPI_IT_MODE)
#error "No SPI interrupt mode!"
#endif
	if(status != HAL_OK) {
		ERR("%s: failed to spi transfer!\n", _name);
		return -1;
	}

	//TODO 添加mutex
	//mutex_unlock(&d->i2c_mutex);
	return 0;
}

s32 spi::self_test(void)
{

	return 0;
}

void spi::isr(void)
{
	HAL_SPI_IRQHandler((SPI_HandleTypeDef *)_handle);
}


#ifdef __cplusplus
extern "C" {
#endif

#if (SPI_MODE == SPI_IT_MODE || SPI_MODE == SPI_DMA_MODE)

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle.
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//post txrx_event
	spi *pspi = NULL;
	for (s32 id = 0; id < ARRAYSIZE(spi_hw_table); id++) {
		if (spi_hw_table[id].pspi->_id == id) {
			pspi = spi_hw_table[id].pspi;
		}
	}

	if (pspi != NULL) {
		pspi->_eventtxrx = 1;
	}

}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	spi *pspi = NULL;
	for (s32 id = 0; id < ARRAYSIZE(spi_hw_table); id++) {
		if (spi_hw_table[id].pspi->_id == id) {
			pspi = spi_hw_table[id].pspi;
		}
	}

	if (pspi != NULL) {
	}

}
#endif

#ifdef __cplusplus
}
#endif

}
/***********************************************************************
** End of file
***********************************************************************/

