/*******************************Copyright (c)***************************
** 
** Porject name:	robot
** Created by:		
** Created date:	
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "rbt_system.h"

using namespace app;

/* STM32F407xG */
s32 main(const s8* argv[], s32 argc)
{
	s32 ret = 0;
	
	rbt_system rbt_system;

	ret = rbt_system.init();
	if (ret < 0)
	{
		INF("Failed to initialize the Robot system.\n");
		return -1;
	}

	rbt_system::get_instance()->start();

	rbt_system::get_instance()->exit();

	return 0;
}
/***********************************************************************
** End of file
***********************************************************************/