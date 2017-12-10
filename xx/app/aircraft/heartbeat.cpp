/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/20
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "heartbeat.h"

#include "leader_system.h"

namespace app {

heartbeat::heartbeat(void)
{
	_params.name = "heartbeat";
	_params.priority = 0;
	_params.stacksize = 128;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

heartbeat::~heartbeat(void)
{

}

void heartbeat::run(void *parg)
{
    gpio *led_amber = leader_system::get_instance()->get_led_amber();

	for (u32 cnt = 0; ;cnt++)
	{
		INF("%s: LEADER_UAV_HEART_BEAT[%u sec]...\n", _os_name, cnt);
		
        // TODO获取CPU使用率

        led_amber->set_value(VHIGH);
        msleep(500);
        led_amber->set_value(VLOW);
        msleep(500);
	}
}

}
/***********************************************************************
** End of file
***********************************************************************/



