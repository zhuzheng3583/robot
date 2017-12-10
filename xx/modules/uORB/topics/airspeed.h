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

/**
 * Airspeed
 */
struct airspeed_s {
	uint64_t	timestamp;			/**< microseconds since system boot, needed to integrate */
	float		indicated_airspeed_m_s;		/**< indicated airspeed in meters per second, -1 if unknown	 */
	float		true_airspeed_m_s;		/**< true airspeed in meters per second, -1 if unknown */
	float		air_temperature_celsius;	/**< air temperature in degrees celsius, -1000 if unknown */
};

/***********************************************************************
** End of file
***********************************************************************/

