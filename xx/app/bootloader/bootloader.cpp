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
#include "bootloader.h"
#include "leader_system.h"

using namespace driver;

namespace app {

bootloader::bootloader(void)
{

}

bootloader::~bootloader(void)
{

}


s32 bootloader::init(void)
{
	leader_system::init();
	INF("========Init LeaderUAV Bootloader ========\n");
	
	_pflash = leader_system::get_instance()->get_flash();

#if 1
	_pflash->open();
	bootloader::iap_upload_firmware(FLASH_APP_START_ADDR);
#endif

	return 0;
}

void bootloader::start(void)
{
	INF("Starting LeaderUAV Bootloader...\n");

	_pflash->open();
	bootloader::iap_upload_firmware(FLASH_APP_START_ADDR);

	_pflash->close();
	delete _pflash;
}

/**
 *  在指定地址开始写入固件，格式为bin
 *  start_addr 用户代码起始地址
 *  buf:应用程序CODE.
 *  len:应用程序大小(字节).
 */
s32 bootloader::iap_download_firmware(u32 start_addr, u8 *buf, u32 len)
{
	s32 ret = 0;
	_pflash->seek(ADDR_FLASH_SECTOR_TO_OFFSET(start_addr), SEEK_SET_M);
	ret = _pflash->write(buf, len);
	if (ret != len) {
		ERR("Failed to %s.\n", __FUNCTION__);
	}

	INF("%s: start_addr=0x%08x, len=%d.\n", __FUNCTION__, start_addr, len);
	return ret;
}

/**
 *  跳转到应用程序段并执行
 *  start_addr 用户代码起始地址
 */
void bootloader::iap_upload_firmware(u32 start_addr)
{
	u32 jump_addr;
	//u32 start_addr = OFFSET_TO_ADDR_FLASH_PAGE(flash_offset);
	INF("%s: start_addr=0x%08x.\n", __FUNCTION__, start_addr);
	INF("%s: SCB->VTOR=0x%08x.\n", __FUNCTION__, SCB->VTOR);
	if(((*(volatile u32*)start_addr)&0x2FFE0000)==0x20000000)		//检查栈顶地址是否合法.
	{ 
	#if 0
		_jump2app = (iap_func)*(volatile u32*)(start_addr + 4);	//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(volatile u32*)start_addr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		INF("%s: success jump to app.\n", __FUNCTION__);
		_jump2app();											//跳转到APP.
	#else
		/* Jump to user application */
   		jump_addr = *(volatile u32*)(start_addr + 4);
    		_jump2app = (iap_func)jump_addr;
    		/* Initialize user application's Stack Pointer */
		__set_MSP(*(volatile u32*) start_addr);
    		_jump2app();
	#endif
	}	
}

}


/***********************************************************************
** End of file
***********************************************************************/

