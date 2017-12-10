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


/* Indicates in which mode the vtol aircraft is in */
struct vtol_vehicle_status_s {

	uint64_t	timestamp;	/**< Microseconds since system boot */
	bool vtol_in_rw_mode;	/*true: vtol vehicle is in rotating wing mode */
	bool fw_permanent_stab;	/**< In fw mode stabilize attitude even if in manual mode*/
	float airspeed_tot;		/*< Estimated airspeed over control surfaces */
};

/***********************************************************************
** End of file
***********************************************************************/