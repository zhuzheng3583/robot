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

struct vehicle_force_setpoint_s {
	float x;		/**< in N NED			  		*/
	float y;		/**< in N NED			  		*/
	float z;		/**< in N NED			  		*/
	float yaw;		/**< right-hand rotation around downward axis (rad, equivalent to Tait-Bryan yaw) */
}; /**< Desired force in NED frame */

/***********************************************************************
** End of file
***********************************************************************/