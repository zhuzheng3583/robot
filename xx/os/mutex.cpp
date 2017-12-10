/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "mutex.h"


namespace os {

mutex::mutex(void)
{

}

mutex::~mutex(void)
{

}

BOOL mutex::create(PCSTR os_name)
{
    _os_name = os_name;
	_os_handle =(HANDLE)osMutexCreate(NULL);
    if (_os_handle == NULL) {
		OS_ERR("_handle = %d.\n", _os_handle);
		return false;
    }

    OS_DBG("mutex create success.\n");
    return true;
}

BOOL mutex::m_delete(void)
{
	osStatus status = osOK;
	status = osMutexDelete(osMutexId(_os_handle));
	if (status != osOK) {
		OS_ERR("status = %d.\n", status);
		return false;
	}

	return true;
}

BOOL mutex::pend(s32 timeoutms)
{
	osStatus status = osOK;
	status = (osStatus)osMutexWait(osMutexId(_os_handle), (uint32_t)timeoutms);
	if (status != osOK) {
		OS_ERR("status = %d.\n", status);
		return false;
	}

	return true;
}

BOOL mutex::post(s32 timeoutms)
{
	osStatus status = osOK;
	status = (osStatus)osMutexRelease(osMutexId(_os_handle));
	if (status != osOK) {
		OS_ERR("status = %d.\n", status);
		return false;
	}

	return true;
}

}
/***********************************************************************
** End of file
***********************************************************************/