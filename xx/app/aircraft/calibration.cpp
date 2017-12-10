/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/23
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "calibration.h"

#include "leader_system.h"

namespace app {

calibration::calibration(void)
{
	_params.name = "calibration_thread";
	_params.priority = 0;
	_params.stacksize = 512;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

calibration::~calibration(void)
{

}

void calibration::run(void *parg)
{
	for ( ; ;)
	{
        msleep(2);
	}
}

}
/***********************************************************************
** End of file
***********************************************************************/


