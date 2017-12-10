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


struct vehicle_attitude_s {
	u64 timestamp;
	f32 roll;
	f32 pitch;
	f32 yaw;

	f32 rollspeed;
	f32 pitchspeed;
	f32 yawspeed;
	f32 rollacc;
	f32 pitchacc;
	f32 yawacc;
};

/***********************************************************************
** End of file
***********************************************************************/

