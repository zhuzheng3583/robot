/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/06/12
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

#include "usb_dev.h"
#include "sensorhub_type.h"

#include "stdlib.h"
#include "usbd_customhid_if.h"

namespace driver {

class sensorhub : public device
{
public:
	sensorhub(PCSTR name, s32 id);
	~sensorhub(void);

public:
    usb_dev *_usb_dev;

    sensorhub_event_t *_in_buf_data;
    u32 _in_buf_size;
    sensorhub_event_t *_out_buf_data;
    u32 _out_buf_size;

public:
    s32 probe(usb_dev *pusb_dev);
    s32 remove(void);
    
public:
    s32 read(void);
    s32 report(void);

public:
    s32 self_test(void);

public:
    virtual s32 read(u8 *buf, u32 size);
    virtual s32 write(u8 *buf, u32 size);
};

}

/***********************************************************************
** End of file
***********************************************************************/
