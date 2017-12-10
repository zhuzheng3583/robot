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

class semaphore : public os_object
{
public:
	semaphore(void);
	~semaphore(void);

public:
	u32 _semcnt;

public:
	BOOL create(PCSTR os_name, u32 semcnt);
	BOOL s_delete(void);
	BOOL pend(s32 timeoutms);
	BOOL post(s32 timeoutms);
	void reset(void);

#if 0
public:
	static semaphore *create(PCSTR name, u32 semcnt);
	static s32 s_delete(semaphore *psem);
	static s32 pend(semaphore *psem, s32 timeoutms);
	static s32 post(semaphore *psem, s32 timeoutms);
	static void reset(semaphore *psem);
#endif
};

}
/***********************************************************************
** End of file
***********************************************************************/
