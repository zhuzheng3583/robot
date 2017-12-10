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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "device.h"

#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_customhid_if.h"

namespace driver {

class usb_dev : public device
{
public:
	usb_dev(PCSTR name, u32 id);
	~usb_dev(void);

public:
    u32 _epin_size;
    u32 _epout_size;

public:
    s32 probe(void);
    s32 remove(void);

public:
    s32 self_test(void);

public:
    virtual s32 read(u8 *buf, u32 size);
    virtual s32 write(u8 *buf, u32 size);
};

}


#ifdef __cplusplus
extern "C" {
#endif

void usb_receive_process(u8 ep_addr, u8 *pbuf, u16 size);

#ifdef __cplusplus
}
#endif

/***********************************************************************
** End of file
***********************************************************************/
