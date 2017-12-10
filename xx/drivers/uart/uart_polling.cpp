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
#include "uart_polling.h"
#include "core.h"

#define UART_POLLING_TIMEOUT_MS 	1000

namespace driver {

//id = [1,3]
uart_polling::uart_polling(PCSTR devname, s32 devid) :
	uart(devname, devid)
{

}

uart_polling::~uart_polling(void)
{

}

s32 uart_polling::probe(void)
{
	s32 ret = 0;
	ret = uart::probe();
	if (ret < 0) {
		goto fail0;
	}
	//INF("%s: probe success.\n", _name);
	return 0;
	
fail0:
	return -1;
}

s32 uart_polling::remove(void)
{
	s32 ret = 0;
	ret = uart::remove();
	if (ret < 0) {
		goto fail0;
	}

	return 0;

fail0:
    return -1;
}

s32 uart_polling::recv(u8 *buf, u32 count)
{
    u32 readcnt = 0;

	if(HAL_UART_Receive((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count, UART_POLLING_TIMEOUT_MS) != HAL_OK) {
    		//CAPTURE_ERR();
	}
    readcnt = count;

	return readcnt;
}

s32 uart_polling::send(u8 *buf, u32 count)
{
    u32 writecnt = 0;

	if(HAL_UART_Transmit((UART_HandleTypeDef*)_handle, (uint8_t*)buf, count, UART_POLLING_TIMEOUT_MS) != HAL_OK) {
        //CAPTURE_ERR();
	}
    writecnt = count;

	return writecnt;
}

s32 uart_polling::read(u8 *buf, u32 size)
{
	return uart_polling::recv(buf, size);
}

s32 uart_polling::write(u8 *buf, u32 size)
{
	return uart_polling::send(buf, size);
}

}



/***********************************************************************
** End of file
***********************************************************************/

