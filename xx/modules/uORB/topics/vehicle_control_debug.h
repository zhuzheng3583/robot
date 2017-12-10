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

struct vehicle_control_debug_s {
	uint64_t timestamp; /**< in microseconds since system start */

	float roll_p;		/**< roll P control part		*/
	float roll_i;		/**< roll I control part 		*/
	float roll_d;		/**< roll D control part 		*/

	float roll_rate_p;	/**< roll rate P control part		*/
	float roll_rate_i;	/**< roll rate I control part 		*/
	float roll_rate_d;	/**< roll rate D control part 		*/

	float pitch_p;		/**< pitch P control part		*/
	float pitch_i;		/**< pitch I control part 		*/
	float pitch_d;		/**< pitch D control part 		*/

	float pitch_rate_p;	/**< pitch rate P control part		*/
	float pitch_rate_i;	/**< pitch rate I control part 		*/
	float pitch_rate_d;	/**< pitch rate D control part 		*/

	float yaw_p;		/**< yaw P control part			*/
	float yaw_i;		/**< yaw I control part 		*/
	float yaw_d;		/**< yaw D control part 		*/

	float yaw_rate_p;	/**< yaw rate P control part		*/
	float yaw_rate_i;	/**< yaw rate I control part 		*/
	float yaw_rate_d;	/**< yaw rate D control part 		*/

}; /**< vehicle_control_debug */

/***********************************************************************
** End of file
***********************************************************************/