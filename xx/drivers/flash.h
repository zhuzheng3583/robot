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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "device.h"

#if 0
#define FLASH_USER_START_ADDR OFFSET_TO_ADDR_FLASH_PAGE(OFFSET_FLASH_PAGE_32)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR   OFFSET_TO_ADDR_FLASH_PAGE(OFFSET_FLASH_PAGE_127)   /* End @ of user Flash area */
#define FLASH_PAGE_SIZE	    0x800                /* Size of page : 2 Kbytes */
#endif

#define OFFSET_TO_ADDR_FLASH_SECTOR(offset)   ((u32)((offset) + FLASH_BASE))
#define ADDR_FLASH_SECTOR_TO_OFFSET(addr)     ((u32)((addr) - FLASH_BASE))

#define DATA_32                             ((u32)0x12345678)

namespace driver {

/* Base address of the Flash sectors */
enum addr_flash_sector {
    ADDR_FLASH_SECTOR_0 = 0x08000000, /* Base @ of Sector 0, 16 Kbytes */
    ADDR_FLASH_SECTOR_1 = 0x08004000, /* Base @ of Sector 1, 16 Kbytes */
    ADDR_FLASH_SECTOR_2 = 0x08008000, /* Base @ of Sector 2, 16 Kbytes */
    ADDR_FLASH_SECTOR_3 = 0x0800C000, /* Base @ of Sector 3, 16 Kbytes */
    ADDR_FLASH_SECTOR_4 = 0x08010000, /* Base @ of Sector 4, 64 Kbytes */
    ADDR_FLASH_SECTOR_5 = 0x08020000, /* Base @ of Sector 5, 128 Kbytes */
    ADDR_FLASH_SECTOR_6 = 0x08040000, /* Base @ of Sector 6, 128 Kbytes */
    ADDR_FLASH_SECTOR_7 = 0x08060000, /* Base @ of Sector 7, 128 Kbytes */
    ADDR_FLASH_SECTOR_8 = 0x08080000, /* Base @ of Sector 8, 128 Kbytes */
    ADDR_FLASH_SECTOR_9 = 0x080A0000, /* Base @ of Sector 9, 128 Kbytes */
    ADDR_FLASH_SECTOR_10 = 0x080C0000, /* Base @ of Sector 10, 128 Kbytes */
    ADDR_FLASH_SECTOR_11 = 0x080E0000, /* Base @ of Sector 11, 128 Kbytes */
};

class flash : public device
{
public:
    flash(PCSTR name, s32 id);
    ~flash(void);

protected:
    u32 _base;
    u32 _offset;
    
public:
    s32 probe(void);
    s32 remove(void);

protected:
    inline s32 erase(u32 start_sector, u32 num_sectors);
    s32 get_sector(u32 address);
    s32 get_sector_num(u32 start_addr, u32 end_addr);
    
public:
    s32 self_test(void);

public:
    virtual s32 open(void);
    virtual s32 read(u8 *buf, u32 size);
    virtual s32 write(u8 *buf, u32 size);
    virtual s32 close(void);

    virtual off_t seek(off_t offset, s32 whence);
	virtual s32 tell(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/

