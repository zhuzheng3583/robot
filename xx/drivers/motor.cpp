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
#include "motor.h"

namespace driver {

motor::motor(PCSTR devname, s32 devid) :
	device(devname, devid),
	_throttle(0),
	_motor_lock(true)
{

}

motor::~motor(void)
{

}

s32 motor::probe(pwm *ppwm, u32 pwm_channel)
{
	ASSERT(ppwm != NULL);

	_ppwm = ppwm;
	_pwm_channel = pwm_channel;

	_ppwm->set_dutycycle(_pwm_channel, 0);
	_ppwm->start(_pwm_channel);

    motor::set_lock(true);
    
	INF("%s: probe success.\n", _devname);
	return 0;

fail0:
	return -1;
}

s32 motor::remove(void)
{
	_ppwm = NULL;
	return 0;

fail0:
    return -1;
}

s32 motor::set_throttle(u32 throttle)
{	
	_throttle = throttle;
	if (_motor_lock == true) {
		_ppwm->set_dutycycle(_pwm_channel, 0);
	}

	_ppwm->set_dutycycle(_pwm_channel, throttle);
		
	return 0;
}

s32 motor::set_lock(bool lock)
{
	_motor_lock = lock;
	if (_motor_lock == true) {
		_ppwm->set_dutycycle(_pwm_channel, 0);
	}

	return 0;
}

}









/***********************************************************************
** End of file
***********************************************************************/


