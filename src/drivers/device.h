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
#pragma once
#include "robot_type.h"
#include "robot_misc.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#define DEV_ERR(fmt, ...) print(3, ("[DEV_ERR] --%s--(%d)-<%s>: %s: " fmt), \
    __FILE__, __LINE__, __FUNCTION__, _devname, ##__VA_ARGS__)
#define DEV_WRN(fmt, ...) print(2, ("[DEV_WRN]<%s>: %s: " fmt), __FUNCTION__, _devname, ##__VA_ARGS__)
#define DEV_INF(fmt, ...) print(1, ("[DEV_INF]<%s>: %s: " fmt), __FUNCTION__, _devname, ##__VA_ARGS__)
#define DEV_DBG(fmt, ...) print(0, ("[DEV_DBG]<%s>: %s: " fmt), __FUNCTION__, _devname, ##__VA_ARGS__)

/***********************************************************************
** End of file
***********************************************************************/
