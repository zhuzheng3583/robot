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

struct thread_params
{
	PCSTR name;								// ��������
	s32	priority;						// �������ȼ�
	u32 stackbase;						// �����ջ����ַ(ע���ֽڶ��룬��ǰϵͳ8�ֽڶ���)
	u32 stacksize;						// �����ջ��С
	void *func;								// ������
	void *parg;								// �����ܲ���
};

class thread : public os_object
{
public:
	thread(void);
	~thread(void);

public:
	struct thread_params _params;

public:
	BOOL create(struct thread_params *pparams);
	BOOL t_delete(void);
	void sleep(s32 timeouts);
	void msleep(s32 timeoutms);
	void usleep(s32 timeoutus);

	u32 get_cpu_usage(void);

public:
	virtual void run(void *parg) = 0;	// ���麯��

protected:
    static BOOL func(thread* pthread);

public:
	inline BOOL set_param(struct thread_params *param);
	inline BOOL get_param(struct thread_params *param) const;
};

}

/***********************************************************************
** End of file
***********************************************************************/

