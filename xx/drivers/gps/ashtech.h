
/* @file ashtech protocol definitions */

#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "device.h"
#include "uart.h"
#include "ringbuffer.h"

#include "vehicle_gps_position.h"
#include "satellite_info.h"

#include "os/thread.h"
#include "mathlib.h"
#include <math.h>

#ifndef RECV_BUFFER_SIZE
#define RECV_BUFFER_SIZE 512
#define SAT_INFO_MAX_SATELLITES  20
#endif

#define GPS_EPOCH_SECS 1234567890ULL

struct tm
{
  int tm_sec;     /* Seconds (0-61, allows for leap seconds) */
  int tm_min;     /* Minutes (0-59) */
  int tm_hour;    /* Hours (0-23) */
  int tm_mday;    /* Day of the month (1-31) */
  int tm_mon;     /* Month (0-11) */
  int tm_year;    /* Years since 1900 */
  int tm_wday;    /* Day of the week (0-6) */
  int tm_yday;    /* Day of the year (0-365) */
  int tm_isdst;   /* Non-0 if daylight savings time is in effect */
};

typedef uint32_t  time_t;         /* Holds time in seconds */

#define CLOCK_REALTIME     0

using namespace os;
using namespace math;

namespace driver {

class ashtech : public device, public thread
{
public:
	ashtech(PCSTR devname, s32 devid);
	~ashtech(void);
  
public:
	enum ashtech_decode_state_t {
		NME_DECODE_UNINIT,
		NME_DECODE_GOT_SYNC1,
		NME_DECODE_GOT_ASTERIKS,
		NME_DECODE_GOT_FIRST_CS_BYTE
	};


	s32 ashtechlog_fd;

	ashtech_decode_state_t	_decode_state;
	u8		_rx_buffer[RECV_BUFFER_SIZE];
	u16   	_rx_buffer_bytes;
	bool 	_parse_error; 		/** parse error flag */
	char 	*_parse_pos; 		/** parse position */

	bool	_gsv_in_progress;			/**< Indicates that gsv data parsing is in progress */
	/* s32     _satellites_count; 			**< Number of satellites info parsed. */
	u8 count;					/**< Number of satellites in satellite info */
	u8 svid[SAT_INFO_MAX_SATELLITES]; 		/**< Space vehicle ID [1..255], see scheme below  */
	u8 used[SAT_INFO_MAX_SATELLITES];		/**< 0: Satellite not used, 1: used for navigation */
	u8 elevation[SAT_INFO_MAX_SATELLITES];	/**< Elevation (0: right on top of receiver, 90: on the horizon) of satellite */
	u8 azimuth[SAT_INFO_MAX_SATELLITES];	/**< Direction of satellite, 0: 0 deg, 255: 360 deg. */
	u8 snr[SAT_INFO_MAX_SATELLITES];		/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite. */

public:    
	u8 _rate_count_lat_lon;
	u8 _rate_count_vel;

	float _rate_lat_lon;
	float _rate_vel;

	uint64_t _interval_rate_start;

public:
	s32             configure(unsigned &baudrate);
	s32             handle_message(s32 len);
	s32             parse_char(u8 b);
	/** Read s32 ashtech parameter */
	s32         read_int();
	/** Read float ashtech parameter */
	double       read_float();
	/** Read char ashtech parameter */
	char            read_char();

public:
    s32 probe(uart *puart);
    s32 remove(void);

public:
	uart *_puart;

	struct vehicle_gps_position_s _gps_position;
	struct satellite_info_s _satellite_info;
	ringbuffer	*_gps_position_reports;
	ringbuffer	*_satellite_info_reports;

public:
	s32 init(void);
	s32 reset(void);
	s32 measure(void);
    
public:
    s32 read_gps_position(u8 *buf, u32 size);
    s32 read_satellite_info(u8 *buf, u32 size);
    
public:
	virtual void run(void *parg);
};

}
