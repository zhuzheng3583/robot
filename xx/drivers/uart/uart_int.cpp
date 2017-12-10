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
#include "uart_int.h"

#define UART_IT_TIMEOUT_MS			10

namespace driver {

//id = [1,3]
uart_int::uart_int(PCSTR devname, s32 devid) :
	uart(devname, devid)
{

}

uart_int::~uart_int(void)
{

}

s32 uart_int::probe(void)
{
	s32 ret = 0;
	ret = uart::probe();
	if (ret < 0) {
		goto fail0;
	}

	//_irq = uart_hw_table[_id].IRQn;

	//device::request_irq(_irq, this);
	//device::enable_irq(_irq);

	//INF("%s: probe success.\n", _name);
	return 0;

fail0:
	return -1;
}

s32 uart_int::remove(void)
{
	//device::disable_irq(_irq);

	s32 ret = 0;
	ret = uart::remove();
	if (ret < 0) {
		goto fail0;
	}
	return 0;

fail0:
    return -1;
}

s32 uart_int::recv(u8 *buf, u32 count)
{
    u32 readcnt = 0;

	if(HAL_UART_Receive_IT((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}
#if 0
    //pend _flag_rx
	s32 ret = 0;
	wait_condition_ms(_flag_rx == 1, UART_IT_TIMEOUT_MS, &ret);
	if (ret < 0) {
		return -1;
	} else {
		_flag_rx = 0;
	}
#endif
    readcnt = count;

	return readcnt;
}

s32 uart_int::send(u8 *buf, u32 count)
{
    u32 writecnt = 0;

	if(HAL_UART_Transmit_IT((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count) != HAL_OK) {
		//CAPTURE_ERR();
	}

#if 0
	//pend tx_event
	s32 ret = 0;
	wait_condition_ms(_flag_tx == 1, UART_IT_TIMEOUT_MS, &ret);
	if (ret < 0) {
		return -1;
	} else {
		_flag_tx = 0;
	}
#endif
    writecnt = count;

	return writecnt;
}

s32 uart_int::read(u8 *buf, u32 size)
{
	return uart_int::recv(buf, size);
}

s32 uart_int::write(u8 *buf, u32 size)
{
	return uart_int::send(buf, size);
}

}



/***********************************************************************
** End of file
***********************************************************************/

