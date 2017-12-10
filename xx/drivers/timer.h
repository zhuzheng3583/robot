/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/05
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
#include "dma.h"

typedef  void (*timer_func_t)(void *arg);

namespace driver {

class timer : public device
{
public:
	timer(PCSTR name, s32 id);
	~timer(void);
    
public:
	timer_func_t _func;
	void *_func_arg;

public:
    s32 probe(void);
    s32 remove(void);
    
public:
    s32 set_timeout(u32 ms);
    s32 set_function(timer_func_t func, void *arg);
    s32 start(void);
    s32 stop(void);
    
public:
	s32 self_test(void);

public:
    virtual void isr(void); 
};

}

/***********************************************************************
** End of file
***********************************************************************/

