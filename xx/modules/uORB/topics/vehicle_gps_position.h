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
 * GPS position in WGS84 coordinates.
 */
struct vehicle_gps_position_s {
	uint64_t timestamp_position;			/**< Timestamp for position information */
	int32_t lat;					/**< Latitude in 1E-7 degrees */
	int32_t lon;					/**< Longitude in 1E-7 degrees */
	int32_t alt;					/**< Altitude in 1E-3 meters (millimeters) above MSL  */

	uint64_t timestamp_variance;
	float s_variance_m_s;				/**< speed accuracy estimate m/s */
	float c_variance_rad;				/**< course accuracy estimate rad */
	uint8_t fix_type; 				/**< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.   */

	float eph;					/**< GPS HDOP horizontal dilution of position in m */
	float epv;					/**< GPS VDOP horizontal dilution of position in m */

	unsigned noise_per_ms;				/**< */
	unsigned jamming_indicator;			/**< */

	uint64_t timestamp_velocity;			/**< Timestamp for velocity informations */
	float vel_m_s;					/**< GPS ground speed (m/s) */
	float vel_n_m_s;				/**< GPS ground speed in m/s */
	float vel_e_m_s;				/**< GPS ground speed in m/s */
	float vel_d_m_s;				/**< GPS ground speed in m/s */
	float cog_rad;					/**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
	bool vel_ned_valid;				/**< Flag to indicate if NED speed is valid */

	uint64_t timestamp_time;			/**< Timestamp for time information */
	uint64_t time_utc_usec;				/**< Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0 */

	uint8_t satellites_used;			/**< Number of satellites used */
};

/***********************************************************************
** End of file
***********************************************************************/