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
#include <stdint.h>

/**
 * Battery voltages and status
 */
struct battery_status_s {
	uint64_t	timestamp;		/**< microseconds since system boot, needed to integrate */
	float   	voltage_v;		/**< Battery voltage in volts, 0 if unknown           	 */
	float   	voltage_filtered_v;		/**< Battery voltage in volts, filtered, 0 if unknown  	 */
	float		current_a;		/**< Battery current in amperes, -1 if unknown */
	float		discharged_mah;		/**< Discharged amount in mAh, -1 if unknown	 */
};

/***********************************************************************
** End of file
***********************************************************************/