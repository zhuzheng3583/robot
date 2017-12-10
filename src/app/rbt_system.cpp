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

//#include "bootloader.h"

#include "main.h"


namespace app {

rbt_system *rbt_system::s_pactive_instance = NULL;

rbt_system::rbt_system(void)
{

}

rbt_system::~rbt_system(void)
{
	delete s_pactive_instance;
	s_pactive_instance = NULL;
}

s32 rbt_system::init(enum rbt_system_mode mode)
{


	return 0;
}

s32 rbt_system::init(void)
{
	demo_main();
        
	s32 ret = 0;

	if(ret < 0) {
		INF("failed to rbt_system::init");
		CAPTURE_ERR();
	}

	return 0;
}

void rbt_system::start(void)
{
}

s32 rbt_system::exit(void)
{
	return 0;
}

}


/***********************************************************************
** End of file
***********************************************************************/

