/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "kernel.h"

namespace os {

class msgque : public os_object
{
public:
	msgque(void);
	~msgque(void);

public:
	u32 _msgcnt;

public:
	BOOL create(PCSTR os_name, u32 msgcnt);
	BOOL q_delete(void);
	BOOL pend(void *pmsg, u32 *psize, s32 timeoutms);
	BOOL post(void *pmsg, u32 size, s32 timeoutms);
	void reset(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/
