/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/04/08
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "leader_system.h"

namespace app {

class aircraft : public leader_system, public thread
{
public:
	aircraft(void);
	~aircraft(void);

public:
    s32 _into_calibrate;
    
public:
	s32 init(void);
	void start(void);
	s32 exit(void);
    
public:
	virtual void run(void *parg);
};

}

/***********************************************************************
** End of file
***********************************************************************/


