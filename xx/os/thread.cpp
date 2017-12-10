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
#include "thread.h"

namespace os {

thread::thread(void)
{

}

thread::~thread(void)
{

}

BOOL thread::create(struct thread_params *pparams)
{
    if (pparams != NULL) {
        _params = *pparams;
    }

#if 0
    if (_params.priority > OS_CFG_PRIO_MAX - 3) {
        _params.priority = OS_CFG_PRIO_MAX - 3;
    }
    else if (_params.priority < 3) {
        _params.priority = 3;
    }

    if (_params.stacksize < OS_CFG_STK_SIZE_MIN) {
        _params.stacksize = OS_CFG_STK_SIZE_MIN;
    }
#endif

	_os_name   = _params.name;

	/* Thread definition */
	//osThreadDef(LED3, LED_Thread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    osThreadDef_t params;
    memset(&params, 0, sizeof(params));
    params.name 		= (char *)(_params.name),      		//< Thread name
    params.pthread 	    = (os_pthread)(_params.func),     	//< start address of thread function
    params.tpriority 	= (osPriority)(_params.priority), 	//< initial thread priority
    params.instances 	= (uint32_t)(0),    				//< maximum number of instances of that thread function
    params.stacksize 	= (uint32_t)(_params.stacksize), 	//< stack size requirements in bytes; 0 is default stack size

	/* Start thread */
	_os_handle = (HANDLE)osThreadCreate(&params, (void *)(_params.parg));
    if (_os_handle == NULL) {
		OS_ERR("_handle = %d.\n", _os_handle);
		return false;
    }

    OS_DBG("thread create success.\n");
    return true;
}

BOOL thread::t_delete(void)
{
	osStatus status = osOK;

	status = osThreadTerminate (osThreadId(_os_handle));
	if (status != osOK) {
		OS_ERR("status = %d.\n", status);
		return false;
	}

	return true;
}

/**
 *  ʹ��ǰ����˯��ָ��ʱ��(����ȷ�ӳ٣���ȷ�ӳ�ʹ�ö�ʱ���ж�POST�ź���)
 *  timeoutms ˯��ʱ�䣨��λ��ms��
 *  note 1.��timeoutms>=0ʱ��ʹ��ǰ����˯��timeoutmsʱ�䣬ֱ����ʱ�򱻻���
 *		 2.��timeoutms<0ʱ��ʹ��ǰ��������˯��(���˯��0xffffffff tich)��ֱ��������
 */
void thread::msleep(s32 timeoutms)
{
	osStatus status = osOK;

	uint32_t millisec = 0;
	if (timeoutms < 0) {
		// note:��ms < 0ʱ����������osDelay���ӳٲ���ת��
		/* TODO: �����ӳ٣�֪�������� */
		millisec = portMAX_DELAY;
	}
	else {
		millisec = (uint32_t)timeoutms;
	}
	status = osDelay (millisec);
	if (status != osOK) {
		OS_ERR("status = %d.\n", status);
		return;
	}

	return;
}

BOOL thread::func(thread* pthread)
{
#if 0
	INF("== START ====================>\n");
	INF("   name: %s.\n",          pthread->_params.name);
	INF("   priority: %d.\n",      pthread->_params.priority);
	INF("   stackbase: 0x%08x.\n", pthread->_params.stackbase);
	INF("   stacksize: 0x%08x.\n", pthread->_params.stacksize);
	INF("== END <======================\n");
#endif
	pthread->run(pthread->_params.parg);

	return true;
}

void thread::run(void *parg)
{
	OS_ERR("Pure virtual function called: %s(...).\n", __FUNCTION__);
	ASSERT(0);
}


BOOL thread::set_param(struct thread_params *param)
{
    if (param == NULL) return false;
    _params = *param;
    return true;
}

BOOL thread::get_param(struct thread_params *param) const
{
    if (param == NULL) return false;
    *param = _params;
    return true;
}

}

/***********************************************************************
** End of file
***********************************************************************/

