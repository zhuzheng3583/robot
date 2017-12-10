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
 * GNSS Satellite Info.
 */

#define SAT_INFO_MAX_SATELLITES  20

struct satellite_info_s {
	uint64_t timestamp;				/**< Timestamp of satellite info */
	uint8_t count;					/**< Number of satellites in satellite info */
	uint8_t svid[SAT_INFO_MAX_SATELLITES]; 		/**< Space vehicle ID [1..255], see scheme below  */
	uint8_t used[SAT_INFO_MAX_SATELLITES];		/**< 0: Satellite not used, 1: used for navigation */
	uint8_t elevation[SAT_INFO_MAX_SATELLITES];	/**< Elevation (0: right on top of receiver, 90: on the horizon) of satellite */
	uint8_t azimuth[SAT_INFO_MAX_SATELLITES];	/**< Direction of satellite, 0: 0 deg, 255: 360 deg. */
	uint8_t snr[SAT_INFO_MAX_SATELLITES];		/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite. */
};

/**
 * NAV_SVINFO space vehicle ID (svid) scheme according to u-blox protocol specs
 * u-bloxM8-V15_ReceiverDescriptionProtocolSpec_Public_(UBX-13003221).pdf
 *
 * GPS		1-32
 * SBAS		120-158
 * Galileo	211-246
 * BeiDou	159-163, 33-64
 * QZSS		193-197
 * GLONASS	65-96, 255
 *
 */

/***********************************************************************
** End of file
***********************************************************************/

