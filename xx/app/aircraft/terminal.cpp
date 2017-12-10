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
#include "terminal.h"

#include "leader_system.h"

namespace app {

terminal::terminal(void)
{
	_params.name = "terminal";
	_params.priority = 0;
	_params.stacksize = 128;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

terminal::~terminal(void)
{

}

void terminal::run(void *parg)
{
	for (u32 cnt = 0; ;cnt++)
	{
		/* TODO:÷’∂ÀΩªª• */
        msleep(100);
		//DBG("%s: task is active[%u]...\n", _os_name, cnt);
        //core::mdelay(100);

	}
}

}
/***********************************************************************
** End of file
***********************************************************************/

