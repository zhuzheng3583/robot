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
#pragma once
#include "uart.h"
#include "core.h"

namespace driver {

class uart_polling : public uart
{
public:
    uart_polling(PCSTR devname, s32 devid);
    ~uart_polling(void);

public:
    virtual s32 probe(void);
    virtual s32 remove(void);

    inline s32 recv(u8 *buf, u32 count);
    inline s32 send(u8 *buf, u32 count);

public:
    virtual s32 read(u8 *buf, u32 size);
    virtual s32 write(u8 *buf, u32 size);
};

}
/***********************************************************************
** End of file
***********************************************************************/
