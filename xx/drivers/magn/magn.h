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
 * mag report structure.  Reads from the device must be in multiples of this
 * structure.
 *
 * Output values are in gauss.
 */
struct mag_report {
	u64 timestamp;
	u64 error_count;
	f32 x;
	f32 y;
	f32 z;
	f32 range_ga;
	f32 scaling;
	f32 temperature;

	s16 x_raw;
	s16 y_raw;
	s16 z_raw;
};

/** mag scaling factors; Vout = (Vin * Vscale) + Voffset */
struct mag_scale {
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

