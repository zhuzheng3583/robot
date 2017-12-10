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


struct vehicle_bodyframe_speed_setpoint_s {
	uint64_t timestamp;		/**< in microseconds since system start, is set whenever the writing thread stores new data */

	float vx;		/**< in m/s				  		*/
	float vy;		/**< in m/s				  		*/
//	float vz;		/**< in m/s				  		*/
	float thrust_sp;
	float yaw_sp;	/**< in radian		-PI +PI		*/
}; /**< Speed in bodyframe to go to */

/***********************************************************************
** End of file
***********************************************************************/