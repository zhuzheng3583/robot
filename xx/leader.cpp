/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/11
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "leader_type.h"
#include "leader_misc.h"

#include "leader_system.h"

using app::leader_system;
using app::logger;
using driver::uart;

#ifdef __cplusplus
extern "C" {
#endif

#define FMT_MAX_CNT	256

int	print(s32 level, const char* fmt, ...)
{
	s32	count = 0;

	va_list ap;
	va_start(ap, fmt);

#if 0
	static u8 str[FMT_MAX_CNT];
	uart* puart2 = leader_system::get_instance()->get_uart2();
	count = vsnprintf((char *)str, FMT_MAX_CNT, fmt, ap);
	str[count-1] = '\r';
	str[count-0] = '\n';
	count = puart2->write(str, count+1);
#else
    logger *plogger = leader_system::get_instance()->get_logger();
	count = plogger->vprintf(fmt, ap);
	//plogger->flush();
#endif
	va_end(ap);

	return count;
}

#ifdef __cplusplus
}
#endif
/***********************************************************************
** End of file
***********************************************************************/