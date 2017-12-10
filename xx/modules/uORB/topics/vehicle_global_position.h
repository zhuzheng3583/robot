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
 * Fused global position in WGS84.
 *
 * This struct contains global position estimation. It is not the raw GPS
 * measurement (@see vehicle_gps_position). This topic is usually published by the position
 * estimator, which will take more sources of information into account than just GPS,
 * e.g. control inputs of the vehicle in a Kalman-filter implementation.
 */
struct vehicle_global_position_s {
	uint64_t timestamp;		/**< Time of this estimate, in microseconds since system start		*/
	uint64_t time_utc_usec;		/**< GPS UTC timestamp in microseconds					   */
	double lat;			/**< Latitude in degrees							 	   */
	double lon;			/**< Longitude in degrees							 	   */
	float alt;			/**< Altitude AMSL in meters						 	   */
	float vel_n; 			/**< Ground north velocity, m/s				 			   */
	float vel_e;			/**< Ground east velocity, m/s							   */
	float vel_d;			/**< Ground downside velocity, m/s						   */
	float yaw; 			/**< Yaw in radians -PI..+PI.							   */
	float eph;			/**< Standard deviation of position estimate horizontally */
	float epv;			/**< Standard deviation of position vertically */
	float terrain_alt;		/**< Terrain altitude in m, WGS84 */
	bool terrain_alt_valid;		/**< Terrain altitude estimate is valid */
};

/***********************************************************************
** End of file
***********************************************************************/