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

/**
 * Fused local position in NED.
 */
struct vehicle_vicon_position_s {
	uint64_t timestamp;			/**< time of this estimate, in microseconds since system start */
	bool valid;				/**< true if position satisfies validity criteria of estimator */

	float x;				/**< X positin in meters in NED earth-fixed frame */
	float y;				/**< X positin in meters in NED earth-fixed frame */
	float z;				/**< Z positin in meters in NED earth-fixed frame (negative altitude) */
	float roll;
	float pitch;
	float yaw;

	// TODO Add covariances here

};

/***********************************************************************
** End of file
***********************************************************************/