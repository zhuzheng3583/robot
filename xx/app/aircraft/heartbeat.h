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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "os/thread.h"

#include "drivers/core.h"



using namespace os;

namespace app {

class heartbeat : public thread
{
public:
	heartbeat(void);
	~heartbeat(void);

public:
	virtual void run(void *parg);
};

}

/***********************************************************************
** End of file
***********************************************************************/




