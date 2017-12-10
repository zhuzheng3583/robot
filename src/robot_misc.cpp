/*******************************Copyright (c)***************************
**
** Porject name:	robot
** Created by:		
** Created date:	
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "robot_type.h"
#include "robot_misc.h"

#include "rbt_system.h"

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