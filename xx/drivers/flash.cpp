/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/06
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "flash.h"

//#include "core.h"

namespace driver {


flash::flash(PCSTR name, s32 id) :
	device(name, id),
    _base(0),
    _offset(0)
{

}

flash::~flash(void)
{

}

s32 flash::probe(void)
{
	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}
	
	INF("%s: probe success.\n", _name);
	return 0;

fail0:
	return -1;	
}

s32 flash::remove(void)
{
	if (device::remove() < 0) {
		goto fail0;
	}

	return 0;

fail0:
    return -1;	
}



s32 flash::open(void)
{
	/* Unlock the Program memory */
	HAL_FLASH_Unlock();

	/* Clear all FLASH flags */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR 
		| FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);
  
	/* Unlock the Program memory */
	HAL_FLASH_Lock();
	
	return 0;

fail:
    return -1;
}

s32 flash::close(void)
{
	HAL_FLASH_Lock(); 
	return 0;
}

s32 flash::erase(u32 start_sector, u32 num_sectors)
{
	u32 sector_error = 0;

	FLASH_EraseInitTypeDef EraseInitStruct;
	memset(&EraseInitStruct, 0, sizeof(EraseInitStruct));
  	/* Fill EraseInit structure*/
  	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  	EraseInitStruct.Sector = start_sector;
  	EraseInitStruct.NbSectors = num_sectors;
  	if(HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error) != HAL_OK) { 
	    /* 
		Error occurred while sector erase. 
		User can add here some code to deal with this error. 
		SectorError will contain the faulty sector and then to know the code 
			error on this sector,
		user can call function 'HAL_FLASH_GetError()'
		*/
    		INF("%s: failed to erase.\n", _name);
		return -1;
  	}

	/* Note: If an erase operation in Flash memory also concerns data in the 
	data or instruction cache, you have to make sure that these data are rewritten before they are 
	accessed during code execution. If this cannot be done safely, it is recommended to flush the 
	caches by setting the DCRST and ICRST bits in the FLASH_CR register. */
  	__HAL_FLASH_DATA_CACHE_DISABLE();
  	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  	__HAL_FLASH_DATA_CACHE_RESET();
 	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

  	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  	__HAL_FLASH_DATA_CACHE_ENABLE();
	
	return 0;
}

s32 flash::read(u8 *buf, u32 size)
{
	s32 ret = 0;
	u32 start_addr = 0;
	u32 end_addr = 0;
	u32 cur_addr = 0;
	u32 page_n = 0;
	u8 *data = (u8 *)buf;
	u32 readcnt = 0;
	
	/* Unlock the Flash to enable the flash control register access */ 
	HAL_FLASH_Unlock();

	start_addr = OFFSET_TO_ADDR_FLASH_SECTOR(_offset);
	end_addr = start_addr + size;
	ASSERT((start_addr >= FLASH_BASE) && (end_addr <= FLASH_END));

	cur_addr = start_addr;
    for (readcnt = 0; readcnt < size; ) {
		data[readcnt++] = *(volatile u8*)cur_addr++;
		_offset++;
    }

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) */
	HAL_FLASH_Lock(); 

	return readcnt;
}

s32 flash::write(u8 *buf, u32 size)
{
	s32 ret = 0;
	u32 start_addr = 0;
	u32 end_addr = 0;
	u32 cur_addr = 0;
	u32 page_n = 0;
	u8 *data = (u8 *)buf;
	u32 writecnt = 0;
	
	/* Unlock the Flash to enable the flash control register access */ 
	HAL_FLASH_Unlock();

	start_addr = OFFSET_TO_ADDR_FLASH_SECTOR(_offset);
	end_addr = start_addr + size;
	ASSERT((start_addr >= FLASH_BASE) && (end_addr <= FLASH_END));

	if((get_sector(start_addr) < 0) && (get_sector_num(start_addr, end_addr) < 0)) {
		return -1;
	}

	ret = flash::erase((u32)get_sector(start_addr), 
		(u32)get_sector_num(start_addr, end_addr));
	if (ret < 0) {
		ERR("%s: failed to flash::erase.\n", _name);
		return -1;
	}

	cur_addr = start_addr;
    for (writecnt = 0; writecnt < size; ) {
	    	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, cur_addr, *((u32 *)(data + writecnt))) == HAL_OK) {
			_offset += 4;
			cur_addr += 4;
			writecnt += 4;
	    	} else {
			/* Error occurred while writing data in Flash memory. 
			User can add here some code to deal with this error */
			ERR("%s: failed to flash::erase.\n", _name);
			return writecnt;
	    	}
    }

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) */
	HAL_FLASH_Lock(); 

	return writecnt;
}


off_t flash::seek(off_t offset, s32 whence)
{
	switch (whence) {
	case SEEK_SET_M:
		_offset = offset;
		break;
	case SEEK_CUR_M:
		break;
	case SEEK_END_M:
		break;
	default:
        break;
	}
	return 0;
}

s32 flash::tell(void)
{
	return _offset;
}

s32 flash::self_test(void)
{
   	s32 ret = 0;
	u8 wbuf[16] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
			0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xAB };
	u8 rbuf[16] = { 0 };

	flash::open();
	flash::seek(ADDR_FLASH_SECTOR_TO_OFFSET(ADDR_FLASH_SECTOR_10), SEEK_SET_M);
	flash::write(wbuf, 16);
	flash::seek(ADDR_FLASH_SECTOR_TO_OFFSET(ADDR_FLASH_SECTOR_10), SEEK_SET_M);
	flash::read(rbuf, 16);
	ret = memcmp((char const *)wbuf, (char const *)rbuf, 16);
	if(ret != 0) {
		INF("%s: failed to flash::self_test.\n", _name);
		CAPTURE_ERR();
	}
	
    return ret;
}

s32 flash::get_sector(u32 address)
{
	s32 sector = 0;

  	if (address < ADDR_FLASH_SECTOR_0) {
		INF("%s: failed to flash::get_sector.\n", _name);
		sector = -1; 
	}
	else if (address < ADDR_FLASH_SECTOR_1) {
		sector = FLASH_SECTOR_0;  
	}
	else if(address < ADDR_FLASH_SECTOR_2) {
    		sector = FLASH_SECTOR_1;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_3) {
    		sector = FLASH_SECTOR_2;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_4) {
    		sector = FLASH_SECTOR_3;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_5) {
    		sector = FLASH_SECTOR_4;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_6) {
    		sector = FLASH_SECTOR_5;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_7) {
    		sector = FLASH_SECTOR_6;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_8) {
    		sector = FLASH_SECTOR_7;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_9) {
    		sector = FLASH_SECTOR_8;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_10) {
    		sector = FLASH_SECTOR_9;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_11) {
    		sector = FLASH_SECTOR_10;  
  	}
  	else {/* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
    		sector = FLASH_SECTOR_11;
  	}

  	return sector;
}

s32 flash::get_sector_num(u32 start_addr, u32 end_addr)
{
#if 0
	s32 sector_num = 0;

	if (start_addr < ADDR_FLASH_SECTOR_4) {
		if (end_addr < ADDR_FLASH_SECTOR_4) {
			
		}
			
	} else if(start_addr < ADDR_FLASH_SECTOR_5) {
		if (end_addr )	
	} 
	
  	if (address < ADDR_FLASH_SECTOR_0) {
		INF("%s: failed to flash::get_sector_num.\n", _name);
		sector = -1; 
	}
	if (address < ADDR_FLASH_SECTOR_1) {
		sector = FLASH_SECTOR_0;  
	}
	else if(address < ADDR_FLASH_SECTOR_2) {
    		sector = FLASH_SECTOR_1;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_3) {
    		sector = FLASH_SECTOR_2;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_4) {
    		sector = FLASH_SECTOR_3;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_5) {
    		sector = FLASH_SECTOR_4;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_6) {
    		sector = FLASH_SECTOR_5;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_7) {
    		sector = FLASH_SECTOR_6;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_8) {
    		sector = FLASH_SECTOR_7;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_9) {
    		sector = FLASH_SECTOR_8;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_10) {
    		sector = FLASH_SECTOR_9;  
  	}
  	else if (address < ADDR_FLASH_SECTOR_11) {
    		sector = FLASH_SECTOR_10;  
  	}
  	else {/* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
    		sector = FLASH_SECTOR_11;
  	}

  	return sector;
	
fail:
	return -1;
#endif
	return 0;
}

}
/***********************************************************************
** End of file
***********************************************************************/

