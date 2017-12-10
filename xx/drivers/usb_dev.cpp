/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/12
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "usb_dev.h"

namespace driver {


usb_dev::usb_dev(PCSTR name, u32 id) : 
	device(name, id),
    _epin_size(CUSTOM_HID_EPIN_SIZE),
    _epout_size(CUSTOM_HID_EPOUT_SIZE)
{
	
}

usb_dev::~usb_dev(void)
{

}

s32 usb_dev::probe(void)
{
	USBD_HandleTypeDef *hUSBDDevice = (USBD_HandleTypeDef *)malloc(sizeof(USBD_HandleTypeDef));
	memset(hUSBDDevice, 0, sizeof(USBD_HandleTypeDef));
	_handle = (u32)hUSBDDevice;
	
	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}

    /* Disconnect the USB device */
    USBD_Stop(hUSBDDevice);
    USBD_DeInit(hUSBDDevice);
	
	/* Init Device Library */
	USBD_Init(hUSBDDevice, &HID_Desc, 0);
  
	/* Add Supported Class */
	USBD_RegisterClass(hUSBDDevice, &USBD_CUSTOM_HID);
  
	/* Add Custom HID callbacks */
	USBD_CUSTOM_HID_RegisterInterface(hUSBDDevice, &USBD_CustomHID_fops);
  
	/* Start Device Process */
	USBD_Start(hUSBDDevice);


  	

	INF("%s: probe success.\n", _name);
	return 0;
	
fail0:
	free(hUSBDDevice);
	return -1;	
}

s32 usb_dev::remove(void) 
{
	USBD_HandleTypeDef *hUSBDDevice = (USBD_HandleTypeDef *)_handle;

	if (device::remove() < 0) {
		goto fail0;
	}
	
	USBD_Stop(hUSBDDevice);
	USBD_DeInit(hUSBDDevice);

	return 0;	

fail0:
	return -1;
}


s32 usb_dev::read(u8 *buf, u32 size)
{
    u32 readcnt = 0;

	readcnt = size;

	return readcnt;
}

s32 usb_dev::write(u8 *buf, u32 size)
{
	USBD_HandleTypeDef *hUSBDDevice = (USBD_HandleTypeDef *)_handle;
	
	u32 writecnt = 0;
	if (size % _epin_size) {
		return -1;
	}

	USBD_CUSTOM_HID_SendReport(hUSBDDevice, buf, size); 

	writecnt = size;
	return writecnt;
}



s32 usb_dev::self_test(void)
{	
	s32 ret = 0;
#if 0
	u8 i = 0;
	u8 buf[CUSTOM_HID_EPIN_SIZE] = { 0 };
	INF("usb_dev::self_test.\n");
	while(1) {  
		USBD_CUSTOM_HID_SendReport(&hUSBDDevice, buf, CUSTOM_HID_EPIN_SIZE); 
       	for (i = 0; i < CUSTOM_HID_EPIN_SIZE; i++) {
          	buf[i]++;
		}
		driver::core::mdelay(20);
	}
#else
	u8 i = 0;
	u8 buf[16] = { 0 };
	while (1) {
		usb_dev::write(buf, 16);
		for (i = 0; i < 16; i++) {
          	buf[i]++;
		}
		driver::core::mdelay(20);
	}
	
#endif
	return ret;

	
  	/* TODO:	1.传输效率(packet_size,缓存) 
			2.是否会丢包(通过传输12345...连续数字)
           	3.传输频率以及延迟(通过时间戳验证)*/

	/*
	#define CUSTOM_HID_EPOUT_SIZE                0x08
	#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE    0x08
	//缓存与SIZE一致时，HOST发给device只需一次发送完成
	*/
}

}

#ifdef __cplusplus
extern "C" {
#endif

void usb_receive_process(u8 ep_addr, u8 *pbuf, u16 size)
{
	//INF("usb_receive_process.\n");
  	//driver::core::mdelay(1000);
	//switch(ep_addr)
	//{
	//case :
	//}
}

#ifdef __cplusplus
}
#endif

/***********************************************************************
** End of file
***********************************************************************/

