#include "ashtech.h"

namespace driver {

ashtech::ashtech(PCSTR devname, s32 devid) :
    device(devname, devid),
    _rate_lat_lon(0.0),
    _rate_vel(0.0)
{
	_decode_state = NME_DECODE_UNINIT;
	_rx_buffer_bytes = 0;

    _params.name = "ashtech_thread";
	_params.priority = 0;
	_params.stacksize = 2048;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

ashtech::~ashtech(void)
{

}

s32 ashtech::probe(uart *puart)
{
	ASSERT(puart != NULL);

    _puart = puart;
	puart->open();

	s32 ret = 0;
	ret = init();
	if (ret) {
		ERR("%s: failed to init.\n", _devname);
        goto fail0;
	}

    return 0;

fail0:
    return -1;
}

s32 ashtech::read_gps_position(u8 *buf, u32 size)
{
	u32 count = size / sizeof(vehicle_gps_position_s);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if no data, error (we could block here) */
	if (_gps_position_reports->empty())
		return -EAGAIN;
    
	/* copy reports out of our buffer to the caller */
	struct vehicle_gps_position_s *position_rp = reinterpret_cast<vehicle_gps_position_s *>(buf);
	int transferred = 0;
	while (count--) {
		if (!_gps_position_reports->get(position_rp))
			break;
		transferred++;
		position_rp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(vehicle_gps_position_s));  
}

s32 ashtech::read_satellite_info(u8 *buf, u32 size)
{
	u32 count = size / sizeof(satellite_info_s);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if no data, error (we could block here) */
	if (_satellite_info_reports->empty())
		return -EAGAIN;
    
	/* copy reports out of our buffer to the caller */
	struct satellite_info_s *satellite_rp = reinterpret_cast<satellite_info_s *>(buf);
	int transferred = 0;
	while (count--) {
		if (!_satellite_info_reports->get(satellite_rp))
			break;
		transferred++;
		satellite_rp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(satellite_info_s));    
}

s32 ashtech::init(void)
{
    if (reset() != 0) {
        ERR("%s: failed to reset.\n", _devname);
        goto out;
    }

    /* allocate basic report buffers */
    _gps_position_reports = new ringbuffer(2, sizeof(struct vehicle_gps_position_s));
    if (_gps_position_reports == NULL) {
        goto out;
    }

    _satellite_info_reports = new ringbuffer(2, sizeof(struct satellite_info_s));
    if (_satellite_info_reports == NULL) {
        goto out;
    }

    /* discard any stale data in the buffers */
    _gps_position_reports->flush();
    _satellite_info_reports->flush();

    //measure();

    return 0;

out:
    return -1;
}

s32 ashtech::reset(void)
{
  
	u8 cfg_rate5HZ[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A };
	_puart->write(cfg_rate5HZ, sizeof(cfg_rate5HZ));
    
    u8 cfg_GPZDA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x60 };
    _puart->write(cfg_GPZDA, sizeof(cfg_GPZDA));
    
    u8 cfg_GPGST[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x07, 0x59 };
    _puart->write(cfg_GPGST, sizeof(cfg_GPGST));
    
    return 0;
}

void ashtech::run(void *parg)
{  
	for (;;) {
        //vTaskSuspendAll();
        //_puart->self_test();
        //xTaskResumeAll();
#if 0
        u8 buf[512];
        
        struct pollfd fds = {
            .events = POLLIN,
        };
    
        //poll(&fds, 200);
		_puart->read(buf, 400);
		INF("%s", buf);
#else
        measure();
        msleep(1000);
#endif
	}
}

s32 ashtech::measure(void)
{
    struct pollfd fds = {
        .events = POLLIN,
    };  

    // $GNRMC,111808.00,A,3106.06235,N,12115.08438,E,0.204,,310716,,,A*6C$GPGSV,4,1,15,02,71,050,23,05,61,314,30,06,34,104,32,07,12,072,*7B$GPGSV,4,2,15,09,02,038,,12,07,237,25,13,44,186,32,15,16,211,21*70$GPGSV,4,3,15,19,16,161,24,20,20,269,,25,05,271,,29,29,314,29*73$GPGSV,4,4,15,30,14,098,,42,46,140,,50,46,140,*49
    u8 buf[200] = "ERR: THIS IS GPS TEST ";
	u32 timeoutms = 2000;

    /* timeout additional to poll */
    timestamp_t timestamp_end = core::get_timestamp() + core::convert_ms_to_timestamp(timeoutms);
	
    s32 j = 0;
    ssize_t bytes_count = 0;

    while (true) {
	    /* pass received bytes to the packet decoder */
	    while (j < bytes_count) {
			s32 l = 0;
            //��⵽ͷβ��־$*��У��ֵ�˶Գɹ��󣬷��ظ�����Ϣ�ֽ�������ʱ����handle_message��ȡ��Ϣ
		    if ((l = parse_char(buf[j])) > 0) {
		        /* return to configure during configuration or to the gps driver during normal work
			     * if a packet has arrived 
                 */
			    if (handle_message(l) > 0) {
                  	_gps_position_reports->force(&_gps_position);
                    _satellite_info_reports->force(&_satellite_info);
				    return 0;
			    }
		    }

		    /* in case we keep trying but only get crap from GPS */
			if (!time_after(timestamp_end, core::get_timestamp())) {
			    return -1;
		    }
		    j++;
	    }

	    /* everything is read */
	    j = bytes_count = 0;
        //poll(&fds, 2000);
	    bytes_count = _puart->read(buf, 16/*sizeof(buf)*/);
        DBG("%s", buf);
    }
}

#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

s32 ashtech::parse_char(u8 b)
{
	s32 ret = 0;

	switch (_decode_state) {
		/* First, look for sync1 */
	case NME_DECODE_UNINIT:
		if (b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;
			_rx_buffer[_rx_buffer_bytes++] = b;
		}

		break;

	case NME_DECODE_GOT_SYNC1:
		if (b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;
		} else if (b == '*') {
			_decode_state = NME_DECODE_GOT_ASTERIKS;
		}

		if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
			_decode_state = NME_DECODE_UNINIT;
			_rx_buffer_bytes = 0;
		} else {
			_rx_buffer[_rx_buffer_bytes++] = b;
		}

		break;

	case NME_DECODE_GOT_ASTERIKS:
		_rx_buffer[_rx_buffer_bytes++] = b;
		_decode_state = NME_DECODE_GOT_FIRST_CS_BYTE;
		break;

	case NME_DECODE_GOT_FIRST_CS_BYTE:
		_rx_buffer[_rx_buffer_bytes++] = b;
		u8 checksum = 0;
		u8 *buffer = _rx_buffer + 1;
		u8 *bufend = _rx_buffer + _rx_buffer_bytes - 3;

		for (; buffer < bufend; buffer++) { 
          checksum ^= *buffer; 
        }

		if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
		    (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
			ret = _rx_buffer_bytes;
		}

		_decode_state = NME_DECODE_UNINIT;
		_rx_buffer_bytes = 0;
		break;
	}

	return ret;
}


/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */
s32 ashtech::handle_message(s32 len)
{
	char * endp;

	if (len < 7) { return 0; }

	s32 uiCalcComma = 0;

	for (s32 i = 0 ; i < len; i++) {
		if (_rx_buffer[i] == ',') { uiCalcComma++; }
	}

	char *bufptr = (char *)(_rx_buffer + 6);

	if ((memcmp(_rx_buffer + 3, "ZDA,", 3) == 0) && (uiCalcComma == 6)) {
		/*
		UTC day, month, and year, and local time zone offset
		An example of the ZDA message string is:

		$GPZDA,172809.456,12,07,1996,00,00*45

		ZDA message fields
		Field	Meaning
		0	Message ID $GPZDA
		1	UTC
		2	Day, ranging between 01 and 31
		3	Month, ranging between 01 and 12
		4	Year
		5	Local time zone offset from GMT, ranging from 00 through 13 hours
		6	Local time zone offset from GMT, ranging from 00 through 59 minutes
		7	The checksum data, always begins with *
		Fields 5 and 6 together yield the total offset. For example, if field 5 is -5 and field 6 is +15, local time is 5 hours and 15 minutes earlier than GMT.
		*/
		double ashtech_time = 0.0;
		s32 day = 0, month = 0, year = 0, local_time_off_hour = 0, local_time_off_min = 0;

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { day = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { month = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { year = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { local_time_off_hour = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { local_time_off_min = strtol(bufptr, &endp, 10); bufptr = endp; }

		s32 ashtech_hour = ashtech_time / 10000;
		s32 ashtech_minute = (ashtech_time - ashtech_hour * 10000) / 100;
		double ashtech_sec = ashtech_time - ashtech_hour * 10000 - ashtech_minute * 100;
		/*
		 * convert to unix timestamp
		 */
		struct tm timeinfo;
		timeinfo.tm_year = year - 1900;
		timeinfo.tm_mon = month - 1;
		timeinfo.tm_mday = day;
		timeinfo.tm_hour = ashtech_hour;
		timeinfo.tm_min = ashtech_minute;
		timeinfo.tm_sec = s32(ashtech_sec);
     
#if 0
		time_t epoch = mktime(&timeinfo);
		if (epoch > GPS_EPOCH_SECS) {
			uint64_t usecs = static_cast<uint64_t>((ashtech_sec - static_cast<uint64_t>(ashtech_sec))) * 1e6;

			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.


			timespec ts;
			ts.tv_sec = epoch;
			ts.tv_nsec = usecs * 1000;
			if (clock_settime(CLOCK_REALTIME, &ts)) {
				WRN("failed setting clock");
			}

			_gps_position.time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position.time_utc_usec += usecs;
		} else {
			_gps_position.time_utc_usec = 0;
		}
#endif
		_gps_position.timestamp_time = core::get_timestamp();//0;//hrt_absolute_time();

	}
    
	else if ((memcmp(_rx_buffer + 3, "GGA,", 3) == 0) && (uiCalcComma == 14)) {
		/*
		  Time, position, and fix related data
		  An example of the GBS message string is:

		  $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F

		  Note - The data string exceeds the ASHTECH standard length.
		  GGA message fields
		  Field   Meaning
		  0   Message ID $GPGGA
		  1   UTC of position fix
		  2   Latitude
		  3   Direction of latitude:
		  N: North
		  S: South
		  4   Longitude
		  5   Direction of longitude:
		  E: East
		  W: West
		  6   GPS Quality indicator:
		  0: Fix not valid
		  1: GPS fix
		  2: Differential GPS fix, OmniSTAR VBS
		  4: Real-Time Kinematic, fixed integers
		  5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
		  7   Number of SVs in use, range from 00 through to 24+
		  8   HDOP
		  9   Orthometric height (MSL reference)
		  10  M: unit of measure for orthometric height is meters
		  11  Geoid separation
		  12  M: geoid separation measured in meters
		  13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
		  14  Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received1.
		  15
		  The checksum data, always begins with *
		  Note - If a user-defined geoid model, or an inclined
		*/
		double ashtech_time = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
		s32 num_of_sv = 0, fix_quality = 0;
		double hdop = 99.9;
		char ns = '?', ew = '?';

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}

		_gps_position.lat = (s32(lat * 0.01) + (lat * 0.01 - s32(lat * 0.01)) * 100.0 / 60.0) * 10000000;
		_gps_position.lon = (s32(lon * 0.01) + (lon * 0.01 - s32(lon * 0.01)) * 100.0 / 60.0) * 10000000;
		_gps_position.alt = alt * 1000;
		_rate_count_lat_lon++;

		if (fix_quality <= 0) {
			_gps_position.fix_type = 0;

		} else {
			/*
			 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
			 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
			 */
			if (fix_quality == 5) { fix_quality = 3; }

			/*
			 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
			 */
			_gps_position.fix_type = 3 + fix_quality - 1;
		}

		_gps_position.timestamp_position = core::get_timestamp();//0;//hrt_absolute_time();

		_gps_position.vel_m_s = 0;                                  /**< GPS ground speed (m/s) */
		_gps_position.vel_n_m_s = 0;                                /**< GPS ground speed in m/s */
		_gps_position.vel_e_m_s = 0;                                /**< GPS ground speed in m/s */
		_gps_position.vel_d_m_s = 0;                                /**< GPS ground speed in m/s */
		_gps_position.cog_rad = 0;                                  /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		_gps_position.vel_ned_valid = true;                         /**< Flag to indicate if NED speed is valid */
		_gps_position.c_variance_rad = 0.1;
		_gps_position.timestamp_velocity = core::get_timestamp();//0;//hrt_absolute_time();
		return 1;

	} else if ((memcmp(_rx_buffer, "$PASHR,POS,", 11) == 0) && (uiCalcComma == 18)) {
		/*
		Example $PASHR,POS,2,10,125410.00,5525.8138702,N,03833.9587380,E,131.555,1.0,0.0,0.007,-0.001,2.0,1.0,1.7,1.0,*34

		    $PASHR,POS,d1,d2,m3,m4,c5,m6,c7,f8,f9,f10,f11,f12,f13,f14,f15,f16,s17*cc
		    Parameter Description Range
		      d1 Position mode 0: standalone
		                       1: differential
		                       2: RTK float
		                       3: RTK fixed
		                       5: Dead reckoning
		                       9: SBAS (see NPT setting)
		      d2 Number of satellite used in position fix 0-99
		      m3 Current UTC time of position fix (hhmmss.ss) 000000.00-235959.99
		      m4 Latitude of position (ddmm.mmmmmm) 0-90 degrees 00-59.9999999 minutes
		      c5 Latitude sector N, S
		      m6 Longitude of position (dddmm.mmmmmm) 0-180 degrees 00-59.9999999 minutes
		      c7 Longitude sector E,W
		      f8 Altitude above ellipsoid +9999.000
		      f9 Differential age (data link age), seconds 0.0-600.0
		      f10 True track/course over ground in degrees 0.0-359.9
		      f11 Speed over ground in knots 0.0-999.9
		      f12 Vertical velocity in decimeters per second +999.9
		      f13 PDOP 0-99.9
		      f14 HDOP 0-99.9
		      f15 VDOP 0-99.9
		      f16 TDOP 0-99.9
		      s17 Reserved no data
		      *cc Checksum
		    */
		bufptr = (char *)(_rx_buffer + 10);

		/*
		 * Ashtech would return empty space as coordinate (lat, lon or alt) if it doesn't have a fix yet
		 */
		s32 coordinatesFound = 0;
		double ashtech_time = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
		s32 num_of_sv = 0, fix_quality = 0;
		double track_true = 0.0, ground_speed = 0.0 , age_of_corr = 0.0;
		double hdop = 99.9, vdop = 99.9, pdop = 99.9, tdop = 99.9, vertic_vel = 0.0;
		char ns = '?', ew = '?';

		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') {
			/*
			 * if a coordinate is skipped (i.e. no fix), it either won't get into this block (two commas in a row)
			 * or strtod won't find anything and endp will point exactly where bufptr is. The same is for lon and alt.
			 */
			lat = strtod(bufptr, &endp);
			if (bufptr != endp) {coordinatesFound++;}
			bufptr = endp;
		}

		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') {
			lon = strtod(bufptr, &endp);
			if (bufptr != endp) {coordinatesFound++;}
			bufptr = endp;
		}

		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') {
			alt = strtod(bufptr, &endp);
			if (bufptr != endp) {coordinatesFound++;}
			bufptr = endp;
		}

		if (bufptr && *(++bufptr) != ',') { age_of_corr = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { track_true = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { ground_speed = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { vertic_vel = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { pdop = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { vdop = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { tdop = strtod(bufptr, &endp); bufptr = endp; }

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}

		_gps_position.lat = (s32(lat * 0.01) + (lat * 0.01 - s32(lat * 0.01)) * 100.0 / 60.0) * 10000000;
		_gps_position.lon = (s32(lon * 0.01) + (lon * 0.01 - s32(lon * 0.01)) * 100.0 / 60.0) * 10000000;
		_gps_position.alt = alt * 1000;
		_rate_count_lat_lon++;

		if (coordinatesFound < 3) {
			_gps_position.fix_type = 0;

		} else {
			_gps_position.fix_type = 3 + fix_quality;
		}

		_gps_position.timestamp_position = core::get_timestamp();//0;//hrt_absolute_time();

		double track_rad = track_true * M_PI / 180.0;

		double velocity_ms = ground_speed  / 1.9438445;			/** knots to m/s */
		double velocity_north = velocity_ms * cos(track_rad);
		double velocity_east  = velocity_ms * sin(track_rad);

		_gps_position.vel_m_s = velocity_ms;                            /**< GPS ground speed (m/s) */
		_gps_position.vel_n_m_s = velocity_north;                       /**< GPS ground speed in m/s */
		_gps_position.vel_e_m_s = velocity_east;                        /**< GPS ground speed in m/s */
		_gps_position.vel_d_m_s = -vertic_vel;                      /**< GPS ground speed in m/s */
		_gps_position.cog_rad = track_rad;                              /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		_gps_position.vel_ned_valid = true;                             /**< Flag to indicate if NED speed is valid */
		_gps_position.c_variance_rad = 0.1;
		_gps_position.timestamp_velocity = core::get_timestamp();//0;//hrt_absolute_time();
		return 1;

	} else if ((memcmp(_rx_buffer + 3, "GST,", 3) == 0) && (uiCalcComma == 8)) {
		/*
		  Position error statistics
		  An example of the GST message string is:

		  $GPGST,172814.0,0.006,0.023,0.020,273.6,0.023,0.020,0.031*6A

		  The Talker ID ($--) will vary depending on the satellite system used for the position solution:

		  $GP - GPS only
		  $GL - GLONASS only
		  $GN - Combined
		  GST message fields
		  Field   Meaning
		  0   Message ID $GPGST
		  1   UTC of position fix
		  2   RMS value of the pseudorange residuals; includes carrier phase residuals during periods of RTK (float) and RTK (fixed) processing
		  3   Error ellipse semi-major axis 1 sigma error, in meters
		  4   Error ellipse semi-minor axis 1 sigma error, in meters
		  5   Error ellipse orientation, degrees from true north
		  6   Latitude 1 sigma error, in meters
		  7   Longitude 1 sigma error, in meters
		  8   Height 1 sigma error, in meters
		  9   The checksum data, always begins with *
		*/
		double ashtech_time = 0.0, lat_err = 0.0, lon_err = 0.0, alt_err = 0.0;
		double min_err = 0.0, maj_err = 0.0, deg_from_north = 0.0, rms_err = 0.0;

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { rms_err = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { maj_err = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { min_err = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { deg_from_north = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { lat_err = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { lon_err = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { alt_err = strtod(bufptr, &endp); bufptr = endp; }

		_gps_position.eph = sqrt(lat_err * lat_err + lon_err * lon_err);
		_gps_position.epv = alt_err;
		_gps_position.s_variance_m_s = 0;
		_gps_position.timestamp_variance = core::get_timestamp();//0;//hrt_absolute_time();

	} else if ((memcmp(_rx_buffer + 3, "GSV,", 3) == 0)) {
		/*
		  The GSV message string identifies the number of SVs in view, the PRN numbers, elevations, azimuths, and SNR values. An example of the GSV message string is:

		  $GPGSV,4,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

		  GSV message fields
		  Field   Meaning
		  0   Message ID $GPGSV
		  1   Total number of messages of this type in this cycle
		  2   Message number
		  3   Total number of SVs visible
		  4   SV PRN number
		  5   Elevation, in degrees, 90 maximum
		  6   Azimuth, degrees from True North, 000 through 359
		  7   SNR, 00 through 99 dB (null when not tracking)
		  8-11    Information about second SV, same format as fields 4 through 7
		  12-15   Information about third SV, same format as fields 4 through 7
		  16-19   Information about fourth SV, same format as fields 4 through 7
		  20  The checksum data, always begins with *
		*/
		/*
		 * currently process only gps, because do not know what
		 * Global satellite ID I should use for non GPS sats
		 */
		bool bGPS = false;

		if (memcmp(_rx_buffer, "$GP", 3) != 0) {
			return 0;

		} else {
			bGPS = true;
		}

		s32 all_msg_num = 0, this_msg_num = 0, tot_sv_visible = 0;
		struct gsv_sat {
			s32 svid;
			s32 elevation;
			s32 azimuth;
			s32 snr;
		} sat[4];
		memset(sat, 0, sizeof(sat));

		if (bufptr && *(++bufptr) != ',') { all_msg_num = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { this_msg_num = strtol(bufptr, &endp, 10); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { tot_sv_visible = strtol(bufptr, &endp, 10); bufptr = endp; }

		if ((this_msg_num < 1) || (this_msg_num > all_msg_num)) {
			return 0;
		}

		if ((this_msg_num == 0) && (bGPS == true)) {
			memset(_satellite_info.svid,     0, sizeof(_satellite_info.svid));
			memset(_satellite_info.used,     0, sizeof(_satellite_info.used));
			memset(_satellite_info.snr,      0, sizeof(_satellite_info.snr));
			memset(_satellite_info.elevation, 0, sizeof(_satellite_info.elevation));
			memset(_satellite_info.azimuth,  0, sizeof(_satellite_info.azimuth));
		}

		s32 end = 4;

		if (this_msg_num == all_msg_num) {
			end =  tot_sv_visible - (this_msg_num - 1) * 4;
			_gps_position.satellites_used = tot_sv_visible;
			_satellite_info.count = SAT_INFO_MAX_SATELLITES;
			_satellite_info.timestamp = 0;//hrt_absolute_time();
		}

		for (s32 y = 0 ; y < end ; y++) {
			if (bufptr && *(++bufptr) != ',') { sat[y].svid = strtol(bufptr, &endp, 10); bufptr = endp; }
			if (bufptr && *(++bufptr) != ',') { sat[y].elevation = strtol(bufptr, &endp, 10); bufptr = endp; }
			if (bufptr && *(++bufptr) != ',') { sat[y].azimuth = strtol(bufptr, &endp, 10); bufptr = endp; }
			if (bufptr && *(++bufptr) != ',') { sat[y].snr = strtol(bufptr, &endp, 10); bufptr = endp; }

			_satellite_info.svid[y + (this_msg_num - 1) * 4]      = sat[y].svid;
			_satellite_info.used[y + (this_msg_num - 1) * 4]      = ((sat[y].snr > 0) ? true : false);
			_satellite_info.snr[y + (this_msg_num - 1) * 4]       = sat[y].snr;
			_satellite_info.elevation[y + (this_msg_num - 1) * 4] = sat[y].elevation;
			_satellite_info.azimuth[y + (this_msg_num - 1) * 4]   = sat[y].azimuth;
		}
	}

	return 0;
}


/*
 * ashtech board configuration script
 */
const char comm[] = "$PASHS,POP,20\r\n"\
	      "$PASHS,NME,ZDA,B,ON,3\r\n"\
	      "$PASHS,NME,GGA,B,OFF\r\n"\
	      "$PASHS,NME,GST,B,ON,3\r\n"\
	      "$PASHS,NME,POS,B,ON,0.05\r\n"\
	      "$PASHS,NME,GSV,B,ON,3\r\n"\
	      "$PASHS,SPD,A,8\r\n"\
	      "$PASHS,SPD,B,9\r\n";

s32 ashtech::configure(unsigned &baudrate)
{
	/* try different baudrates */
	const unsigned baudrates_to_try[] = {9600, 38400, 19200, 57600, 115200};

#if 0
	for (unsigned s32 baud_i = 0; baud_i < sizeof(baudrates_to_try) / sizeof(baudrates_to_try[0]); baud_i++) {
		baudrate = baudrates_to_try[baud_i];
		set_baudrate(_fd, baudrate);
		write(_fd, comm, sizeof(comm));
	}

	set_baudrate(_fd, 115200);
#endif
	return 0;
}

#if 0

////////////////////////////////////////////////////////////////////////////////////////////////////
//UBLOX NEO-6M ����(���,����,���ص�)�ṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG CFG ID:0X0906 (С��ģʽ)
	u16 dlength;				//���ݳ��� 12/13
	u32 clearmask;				//�������������(1��Ч)
	u32 savemask;				//�����򱣴�����
	u32 loadmask;				//�������������
	u8  devicemask; 		  	//Ŀ������ѡ������	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	u8  cka;		 			//У��CK_A
	u8  ckb;			 		//У��CK_B
}_ublox_cfg_cfg;

//UBLOX NEO-6M ��Ϣ���ýṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG MSG ID:0X0106 (С��ģʽ)
	u16 dlength;				//���ݳ��� 8
	u8  msgclass;				//��Ϣ����(F0 ����NMEA��Ϣ��ʽ)
	u8  msgid;					//��Ϣ ID
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	u8  iicset;					//IIC���������    0,�ر�;1,ʹ��.
	u8  uart1set;				//UART1�������	   0,�ر�;1,ʹ��.
	u8  uart2set;				//UART2�������	   0,�ر�;1,ʹ��.
	u8  usbset;					//USB�������	   0,�ر�;1,ʹ��.
	u8  spiset;					//SPI�������	   0,�ر�;1,ʹ��.
	u8  ncset;					//δ֪�������	   Ĭ��Ϊ1����.
 	u8  cka;			 		//У��CK_A
	u8  ckb;			    	//У��CK_B
}_ublox_cfg_msg;

//UBLOX NEO-6M UART�˿����ýṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG PRT ID:0X0006 (С��ģʽ)
	u16 dlength;				//���ݳ��� 20
	u8  portid;					//�˿ں�,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	u8  reserved;				//����,����Ϊ0
	u16 txready;				//TX Ready��������,Ĭ��Ϊ0
	u32 mode;					//���ڹ���ģʽ����,��żУ��,ֹͣλ,�ֽڳ��ȵȵ�����.
 	u32 baudrate;				//����������
 	u16 inprotomask;		 	//����Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	u16 outprotomask;		 	//���Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	u16 reserved4; 				//����,����Ϊ0
 	u16 reserved5; 				//����,����Ϊ0
 	u8  cka;			 		//У��CK_A
	u8  ckb;			    	//У��CK_B
}_ublox_cfg_prt;

//UBLOX NEO-6M ʱ���������ýṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG TP ID:0X0706 (С��ģʽ)
	u16 dlength;				//���ݳ���
	u32 interval;				//ʱ��������,��λΪus
	u32 length;				 	//������,��λΪus
	signed char status;			//ʱ����������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
	u8 timeref;			   		//�ο�ʱ��:0,UTCʱ��;1,GPSʱ��;2,����ʱ��.
	u8 flags;					//ʱ���������ñ�־
	u8 reserved;				//����
 	signed short antdelay;	 	//������ʱ
 	signed short rfdelay;		//RF��ʱ
	signed int userdelay; 	 	//�û���ʱ
	u8 cka;						//У��CK_A
	u8 ckb;						//У��CK_B
}_ublox_cfg_tp;




/////////////////////////////////////////UBLOX ���ô���/////////////////////////////////////
//���CFG����ִ�����
//����ֵ:0,ACK�ɹ�
//       1,���ճ�ʱ����
//       2,û���ҵ�ͬ���ַ�
//       3,���յ�NACKӦ��
u8 ublox_cfg_ack_check(void)
{
	u16 len=0,i;
	u8 rval=0;
	while((USART3_RX_STA & 0X8000) == 0 && len < 100)//�ȴ����յ�Ӧ��
	{
		len++;
		delay_ms(5);
	}
	if(len < 250)   	//��ʱ����.
	{
		len = USART3_RX_STA & 0X7FFF;	//�˴ν��յ������ݳ���
		for(i=0; i < len; i++)
			if(USART3_RX_BUF[i]==0XB5)
				break;//����ͬ���ַ� 0XB5
		if(i == len)
			rval = 2;						//û���ҵ�ͬ���ַ�
		else if(USART3_RX_BUF[i+3] == 0X00)
			rval = 3;//���յ�NACKӦ��
		else
			rval = 0;	   						//���յ�ACKӦ��
	}
	else
	rval = 1;								//���ճ�ʱ����
	USART3_RX_ST A= 0;							//�������
	return rval;
}
//���ñ���
//����ǰ���ñ������ⲿEEPROM����
//����ֵ:0,ִ�гɹ�;1,ִ��ʧ��.
u8 Ublox_Cfg_Cfg_Save(void)
{
	u8 i;
	_ublox_cfg_cfg cfg;
	_ublox_cfg_cfg *cfg_cfg=&cfg;
	cfg_cfg->header=0X62B5;		//cfg header
	cfg_cfg->id=0X0906;			//cfg cfg id
	cfg_cfg->dlength=13;		//����������Ϊ13���ֽ�.
	cfg_cfg->clearmask=0;		//�������Ϊ0
	cfg_cfg->savemask=0XFFFF; 	//��������Ϊ0XFFFF
	cfg_cfg->loadmask=0; 		//��������Ϊ0
	cfg_cfg->devicemask=4; 		//������EEPROM����
	ublox_checksum((u8*)(&cfg_cfg->id),sizeof(_ublox_cfg_cfg)-4,&cfg_cfg->cka,&cfg_cfg->ckb);
	ublox_send_date((u8*)cfg_cfg,sizeof(_ublox_cfg_cfg));//�������ݸ�NEO-6M
	for(i=0;i<6;i++)if(ublox_cfg_ack_check()==0)break;		//EEPROMд����Ҫ�ȽϾ�ʱ��,���������ж϶��
	return i==6?1:0;
}
//����NMEA�����Ϣ��ʽ
//msgid:Ҫ������NMEA��Ϣ��Ŀ,���������Ĳ�����
//      00,GPGGA;01,GPGLL;02,GPGSA;
//		03,GPGSV;04,GPRMC;05,GPVTG;
//		06,GPGRS;07,GPGST;08,GPZDA;
//		09,GPGBS;0A,GPDTM;0D,GPGNS;
//uart1set:0,����ر�;1,�������.
//����ֵ:0,ִ�гɹ�;����,ִ��ʧ��.
u8 Ublox_Cfg_Msg(u8 msgid,u8 uart1set)
{
	_ublox_cfg_msg msg;
	_ublox_cfg_msg *cfg_msg=&msg;
	cfg_msg->header=0X62B5;		//cfg header
	cfg_msg->id=0X0106;			//cfg msg id
	cfg_msg->dlength=8;			//����������Ϊ8���ֽ�.
	cfg_msg->msgclass=0XF0;  	//NMEA��Ϣ
	cfg_msg->msgid=msgid; 		//Ҫ������NMEA��Ϣ��Ŀ
	cfg_msg->iicset=1; 			//Ĭ�Ͽ���
	cfg_msg->uart1set=uart1set; //��������
	cfg_msg->uart2set=1; 	 	//Ĭ�Ͽ���
	cfg_msg->usbset=1; 			//Ĭ�Ͽ���
	cfg_msg->spiset=1; 			//Ĭ�Ͽ���
	cfg_msg->ncset=1; 			//Ĭ�Ͽ���
	ublox_checksum((u8*)(&cfg_msg->id),sizeof(_ublox_cfg_msg)-4,&cfg_msg->cka,&cfg_msg->ckb);
	ublox_send_date((u8*)cfg_msg,sizeof(_ublox_cfg_msg));//�������ݸ�NEO-6M
	return ublox_cfg_ack_check();
}
//����NMEA�����Ϣ��ʽ
//baudrate:������,4800/9600/19200/38400/57600/115200/230400
//����ֵ:0,ִ�гɹ�;����,ִ��ʧ��(���ﲻ�᷵��0��)
u8 ublox_cfg_prt(u32 baudrate)
{
	_ublox_cfg_prt cfg;
	_ublox_cfg_prt *cfg_prt=&cfg;
	cfg_prt->header=0X62B5;		//cfg header
	cfg_prt->id=0X0006;			//cfg prt id
	cfg_prt->dlength=20;		//����������Ϊ20���ֽ�.
	cfg_prt->portid=1;			//��������1
	cfg_prt->reserved=0;	 	//�����ֽ�,����Ϊ0
	cfg_prt->txready=0;	 		//TX Ready����Ϊ0
	cfg_prt->mode=0X08D0; 		//8λ,1��ֹͣλ,��У��λ
	cfg_prt->baudrate=baudrate; //����������
	cfg_prt->inprotomask=0X0007;//0+1+2
	cfg_prt->outprotomask=0X0007;//0+1+2
 	cfg_prt->reserved4=0; 		//�����ֽ�,����Ϊ0
 	cfg_prt->reserved5=0; 		//�����ֽ�,����Ϊ0
	ublox_checksum((u8*)(&cfg_prt->id),sizeof(_ublox_cfg_prt)-4,&cfg_prt->cka,&cfg_prt->ckb);
	ublox_send_date((u8*)cfg_prt,sizeof(_ublox_cfg_prt));//�������ݸ�NEO-6M
	delay_ms(200);				//�ȴ��������
	usart3_init(baudrate);	//���³�ʼ������3
	return ublox_cfg_ack_check();//���ﲻ�ᷴ��0,��ΪUBLOX��������Ӧ���ڴ������³�ʼ����ʱ���Ѿ���������.
}
//����UBLOX NEO-6��ʱ���������
//interval:������(us)
//length:������(us)
//status:��������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
//����ֵ:0,���ͳɹ�;����,����ʧ��.
u8 ublox_cfg_yp(u32 interval,u32 length,signed char status)
{
	_ublox_cfg_tp tp;
	_ublox_cfg_tp *cfg_tp=&tp;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//����������Ϊ20���ֽ�.
	cfg_tp->interval=interval;	//������,us
	cfg_tp->length=length;		//������,us
	cfg_tp->status=status;	   	//ʱ����������
	cfg_tp->timeref=0;			//�ο�UTC ʱ��
	cfg_tp->flags=0;			//flagsΪ0
	cfg_tp->reserved=0;		 	//����λΪ0
	cfg_tp->antdelay=820;    	//������ʱΪ820ns
	cfg_tp->rfdelay=0;    		//RF��ʱΪ0ns
	cfg_tp->userdelay=0;    	//�û���ʱΪ0ns
	ublox_checksum((u8*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	ublox_send_date((u8*)cfg_tp,sizeof(_ublox_cfg_tp));//�������ݸ�NEO-6M
	return ublox_cfg_ack_check();
}
//����UBLOX NEO-6�ĸ�������
//interval:����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
//reftime:�ο�ʱ�䣬0=UTC Time��1=GPS Time��һ������Ϊ1��
//����ֵ:0,���ͳɹ�;����,����ʧ��.
u8 config_rate(u16 interval, u8 reftime)
{
	//UBLOX NEO-6M ˢ���������ýṹ��
	struct ublox_config_rate_packet
	{
	 	u16 header;				//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
		u16 id;					//CFG RATE ID:0X0806 (С��ģʽ)
		u16 dlength;				//���ݳ���
		u16 measrate;				//����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
		u16 navrate;				//�������ʣ����ڣ����̶�Ϊ1
		u16 timeref;				//�ο�ʱ�䣺0=UTC Time��1=GPS Time��
	 	u8  cka;					//У��CK_A
		u8  ckb;					//У��CK_B
	} config_rate_packet;

 	if(interval<200) return 1;	//С��200ms��ֱ���˳�
 	config_rate_packet.header=0X62B5;	//cfg header
	config_rate_packet.id = 0X0806;	 	//cfg rate id
	config_rate_packet.dlength = 6;	 	//����������Ϊ6���ֽ�.
	config_rate_packet.measrate = interval;//������,us
	config_rate_packet.navrate = 1;		//�������ʣ����ڣ����̶�Ϊ1
	config_rate_packet.timeref = reftime; 	//�ο�ʱ��ΪGPSʱ��
	ublox_checksum((u8*)(&config_rate_packet.id), sizeof(config_rate_packet) - 4, &config_rate_packet.cka, &config_rate_packet.ckb);
	//ublox_send_date((u8*)cfg_rate,sizeof(_ublox_cfg_rate));//�������ݸ�NEO-6M
	dev->write(uart_fops, (u8*)cfg_rate, sizeof(_ublox_cfg_rate));
	return ublox_cfg_ack_check();
}
//����һ�����ݸ�Ublox NEO-6M������ͨ������3����
//dbuf�����ݻ����׵�ַ
//len��Ҫ���͵��ֽ���
void ublox_send_date(u8* dbuf,u16 len)
{
	//u16 j;
	//for(j=0;j<len;j++)//ѭ����������
	//{
	//	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������
	//	USART3->DR=dbuf[j];
	//}
}
#endif

}
