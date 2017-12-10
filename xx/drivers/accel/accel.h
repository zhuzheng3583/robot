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


/**
 * accel report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct accel_report {
	u64 timestamp;
	u64 error_count;
	f32 x;		/**< acceleration in the NED X board axis in m/s^2 */
	f32 y;		/**< acceleration in the NED Y board axis in m/s^2 */
	f32 z;		/**< acceleration in the NED Z board axis in m/s^2 */
	f32 temperature;	/**< temperature in degrees celsius */
	f32 range_m_s2;	/**< range in m/s^2 (+- this value) */
	f32 scaling;

	s16 x_raw;
	s16 y_raw;
	s16 z_raw;
	s16 temperature_raw;
};

/** accel scaling factors; Vout = Vscale * (Vin + Voffset) */
struct accel_scale {
	f32	x_offset;
	f32	x_scale;
	f32	y_offset;
	f32	y_scale;
	f32	z_offset;
	f32	z_scale;
};

#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/

/***********************************************************************
** End of file
***********************************************************************/

