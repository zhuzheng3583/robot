/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/08/19
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
#include "pwm.h"

//3,5,8

namespace driver {

class motor : public device
{
public:
	motor(PCSTR devname, s32 devid);
	~motor(void);

public:
    pwm *_ppwm;
    u32 _pwm_channel;
    u32 _throttle;
    bool _motor_lock;
    
public:
    s32 probe(pwm *ppwm, u32 pwm_channel);
    s32 remove(void);

public:
    s32 set_throttle(u32 throttle);
    s32 set_lock(bool lock);

public:
	s32 self_test(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/

