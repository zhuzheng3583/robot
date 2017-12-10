/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/08/16
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"
#include <stdint.h>
#include <stdbool.h>

struct vehicle_land_detected_s {
	uint64_t timestamp;	   /**< timestamp of the setpoint */
	bool landed;           /**< true if vehicle is currently landed on the ground*/
};

/***********************************************************************
** End of file
***********************************************************************/