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
 *  ��ָ����ַ��ʼд��̼�����ʽΪbin
 *  start_addr �û�������ʼ��ַ
 *  buf:Ӧ�ó���CODE.
 *  len:Ӧ�ó����С(�ֽ�).
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
 *  ��ת��Ӧ�ó���β�ִ��
 *  start_addr �û�������ʼ��ַ
 */
void bootloader::iap_upload_firmware(u32 start_addr)
{
	u32 jump_addr;
	//u32 start_addr = OFFSET_TO_ADDR_FLASH_PAGE(flash_offset);
	INF("%s: start_addr=0x%08x.\n", __FUNCTION__, start_addr);
	INF("%s: SCB->VTOR=0x%08x.\n", __FUNCTION__, SCB->VTOR);
	if(((*(volatile u32*)start_addr)&0x2FFE0000)==0x20000000)		//���ջ����ַ�Ƿ�Ϸ�.
	{ 
	#if 0
		_jump2app = (iap_func)*(volatile u32*)(start_addr + 4);	//�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)		
		MSR_MSP(*(volatile u32*)start_addr);					//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
		INF("%s: success jump to app.\n", __FUNCTION__);
		_jump2app();											//��ת��APP.
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

