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
 * baro report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct baro_report {
	float pressure;
	float altitude;
	float temperature;
	uint64_t timestamp;
	uint64_t error_count;
};

/***********************************************************************
** End of file
***********************************************************************/

