/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "uart_dma.h"

#define UART_DMA_TIMEOUT_MS 		10

namespace driver {

//id = [1,3]
uart_dma::uart_dma(PCSTR devname, s32 devid) :
	uart_int(devname, devid)
{

}

uart_dma::~uart_dma(void)
{

}

s32 uart_dma::probe(void)
{
	s32 ret = 0;
	ret = uart_int::probe();
	if (ret < 0) {
		goto fail0;
	}
	UART_HandleTypeDef *huart = (UART_HandleTypeDef *)_handle;
    
	_dma_tx_id = uart_hw_table[_id].dma_tx_id;
	_dma_rx_id = uart_hw_table[_id].dma_rx_id;
    
	s8 str[16];
	snprintf((char *)str, 16, "dma-%d", _dma_tx_id);
	_dmatx = new dma((PCSTR)str, _dma_tx_id);
	//INF("%s: new dmatx %s[dma%d,stream%d,channel%d].\n", _name, str, _dmatx->_dma_id, _dmatx->_stream_id, _dmatx->_channel_id);
	_dmatx->probe();
	_dmatx->config(DMA_DIR_MEM_TO_PERIPH, DMA_ALIGN_BYTE, DMA_PRI_LOW, 1, 0);
	/* Associate the initialized DMA handle to the UART handle */
	__HAL_LINKDMA(huart, hdmatx, *(DMA_HandleTypeDef *)(_dmatx->_handle));

    snprintf((char *)str, 16, "dma-%d", _dma_rx_id);
	_dmarx = new dma((PCSTR)str, _dma_rx_id);
	//INF("%s: new dmarx %s[dma%d,stream%d,channel%d].\n", _name, str, _dmarx->_dma_id, _dmarx->_stream_id, _dmarx->_channel_id);
	_dmarx->probe();
	_dmarx->config(DMA_DIR_PERIPH_TO_MEM, DMA_ALIGN_BYTE, DMA_PRI_HIGH, 0, 1);
	__HAL_LINKDMA(huart, hdmarx, *(DMA_HandleTypeDef *)(_dmarx->_handle));

	//INF("%s: probe success.\n", _name);
	return 0;

fail0:
	return -1;
}

s32 uart_dma::remove(void)
{
	_dmatx->remove();
	_dmarx->remove();
	delete _dmatx;
	delete _dmarx;
	_dmatx = NULL;
	_dmarx = NULL;

	s32 ret = 0;
	ret = uart_int::remove();
	if (ret < 0) {
		goto fail0;
	}
	
	return 0;

fail0:
    return -1;
}

s32 uart_dma::recv(u8 *buf, u32 count)
{
    u32 readcnt = 0;

	if(HAL_UART_Receive_DMA((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
#if 1
	//pend _flag_rx
	s32 ret = 0;
	wait_condition_ms(_flag_rx == 1, UART_DMA_TIMEOUT_MS, &ret);
	if (ret < 0) {
		readcnt = count - _dmarx->get_leftover_count();
	} else {
		_flag_rx = 0;
		readcnt = count;
	}
#else
    readcnt = count;
#endif
    
	return readcnt;
}

s32 uart_dma::send(u8 *buf, u32 count)
{
    u32 writecnt = 0;

	if(HAL_UART_Transmit_DMA((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
#if 1
	//pend tx_event
	s32 ret = 0;
	wait_condition_ms(_flag_tx == 1, UART_DMA_TIMEOUT_MS, &ret);
	if (ret < 0) {
		writecnt = count - _dmatx->get_leftover_count();
	} else {
		_flag_tx = 0;
        writecnt = count;
	}
#else
    writecnt = count;
#endif
	return writecnt;
}

s32 uart_dma::read(u8 *buf, u32 size)
{
	return uart_dma::recv(buf, size);
}

s32 uart_dma::write(u8 *buf, u32 size)
{
	return uart_dma::send(buf, size);
}

}



/***********************************************************************
** End of file
***********************************************************************/

