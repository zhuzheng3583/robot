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
#include "leader_type.h"
#include "leader_misc.h"
#include "leader_system.h"

using namespace app;

/* STM32F407xG */
s32 main(const s8* argv[], s32 argc)
{
	s32 ret = 0;
	
	leader_system leader_system;

	ret = leader_system.init(leader_system::M_AUTO);
	if (ret < 0)
	{
		INF("Failed to initialize the LeaderUAV system.\n");
		return -1;
	}

	leader_system::get_instance()->start();

	leader_system::get_instance()->exit();

	return 0;
}
/***********************************************************************
** End of file
***********************************************************************/