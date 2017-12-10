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
#include "os/semaphore.h"

namespace os {

semaphore::semaphore(void) :
    _semcnt(0)
{

}

semaphore::~semaphore(void)
{

}

BOOL semaphore::create(PCSTR os_name, u32 semcnt)
{
    _os_name = os_name;
    _semcnt = semcnt;
	//osSemaphoreDef(name);
	_os_handle = (HANDLE)osSemaphoreCreate (NULL, (int32_t)semcnt);
    if (_os_handle == NULL) {
		OS_ERR("_handle = %d.\n", _os_handle);
		return false;
    }

    OS_DBG("semaphore create success.\n");
    return true;
}

BOOL semaphore::s_delete(void)
{
	osStatus status = osOK;
	status = osSemaphoreDelete(osSemaphoreId(_os_handle));
	if (status != osOK) {
		OS_ERR("status = %d.\n", status);
		return false;
	}

	return true;
}

BOOL semaphore::pend(s32 timeoutms)
{
	osStatus status = osOK;
	status = (osStatus)osSemaphoreWait(osSemaphoreId(_os_handle), (uint32_t)timeoutms);
	if (status != osOK) {
		OS_ERR("status = %d.\n", status);
		return false;
	}

	return true;
}

BOOL semaphore::post(s32 timeoutms)
{
	osStatus status = osOK;
	status = osSemaphoreRelease(osSemaphoreId(_os_handle));
	if (status != osOK) {
		//OS_ERR("status = %d.\n", status);
		return false;
	}

	return true;
}

void semaphore::reset(void)
{

}

}
/***********************************************************************
** End of file
***********************************************************************/
