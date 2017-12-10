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
#include "cmsis_os.h"

namespace os {

class mutex : public os_object
{
public:
	mutex(void);
	~mutex(void);

public:

public:
	BOOL create(PCSTR os_name);
	BOOL m_delete(void);
	BOOL pend(s32 timeoutms);
	BOOL post(s32 timeoutms);
};

}
/***********************************************************************
** End of file
***********************************************************************/
