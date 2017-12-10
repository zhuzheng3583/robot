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

#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "thread.h"
#include "circbuf.h"
#include "mutex.h"
#include "uart.h"

#define FMT_MAX_CNT							256

using namespace driver;
using namespace os;

namespace app {

class logger : public thread
{
public:
	logger(void);
	~logger(void);

public:
	circbuf *_pcircbuf;
	uart  *_puart;

	u32 _buf_size;
	u32 _buf_threshold;
	s32 _level;
    s32 _task_created;
    
	mutex *_pmutex;

public:
	BOOL create(struct thread_params *pparams);
	BOOL t_delete(void);

	void attach(uart *puart);
	void detach(void)	;

	s32 vprintf(PCSTR fmt, va_list ap);

	void flush(void);

	s32 no_os_self_test(void);	// 检查环形缓冲区是否工作正常,及时刷缓冲区
	s32 self_test(void);		// 通过任务调度刷新缓冲区

public:
	void set_buf_threshold(s32 threshold)	{ _buf_threshold = threshold; }
	s32 get_buf_threshold(void)			{ return _buf_threshold; }

	void set_level(int32_t level)			{ _level = level; }
	s32 get_level(void) 				    { return _level; }


public:
	virtual void run(void *parg);
};

}
/***********************************************************************
** End of file
***********************************************************************/

