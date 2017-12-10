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
 * Vision based position estimate in NED frame
 */
struct vision_position_estimate {

	unsigned id;				/**< ID of the estimator, commonly the component ID of the incoming message */

	uint64_t timestamp_boot;		/**< time of this estimate, in microseconds since system start */
	uint64_t timestamp_computer;		/**< timestamp provided by the companion computer, in us */

	float x;				/**< X position in meters in NED earth-fixed frame */
	float y;				/**< Y position in meters in NED earth-fixed frame */
	float z;				/**< Z position in meters in NED earth-fixed frame (negative altitude) */

	float vx;				/**< X velocity in meters per second in NED earth-fixed frame */
	float vy;				/**< Y velocity in meters per second in NED earth-fixed frame */
	float vz;				/**< Z velocity in meters per second in NED earth-fixed frame */

	float q[4];				/**< Estimated attitude as quaternion */

	// XXX Add covariances here

};

/***********************************************************************
** End of file
***********************************************************************/