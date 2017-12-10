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

/** Wind estimate */
struct wind_estimate_s {

	uint64_t	timestamp;		/**< Microseconds since system boot */
	float		windspeed_north;	/**< Wind component in north / X direction */
	float		windspeed_east;		/**< Wind component in east / Y direction */
	float		covariance_north;	/**< Uncertainty - set to zero (no uncertainty) if not estimated */
	float		covariance_east;	/**< Uncertainty - set to zero (no uncertainty) if not estimated */
};

/***********************************************************************
** End of file
***********************************************************************/