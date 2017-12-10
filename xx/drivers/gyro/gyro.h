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
 * gyro report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct gyro_report {
	u64 timestamp;
	u64 error_count;
	f32 x;		/**< angular velocity in the NED X board axis in rad/s */
	f32 y;		/**< angular velocity in the NED Y board axis in rad/s */
	f32 z;		/**< angular velocity in the NED Z board axis in rad/s */
	f32 temperature;	/**< temperature in degrees celcius */
	f32 range_rad_s;
	f32 scaling;

	s32 x_raw;
	s32 y_raw;
	s32 z_raw;
	s32 temperature_raw;
};

/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */
struct gyro_scale {
	f32	x_offset;
	f32	x_scale;
	f32	y_offset;
	f32	y_scale;
	f32	z_offset;
	f32	z_scale;
};

/***********************************************************************
** End of file
***********************************************************************/

