/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/30
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "ms5611.h"
#include <math.h>

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { s32 _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define MS5611_CONVERSION_INTERVAL	10000	/* microseconds */
#define MS5611_CONVERSION_INTERVAL_MS	10	/* microseconds */

#define MS5611_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

namespace driver {

ms5611::ms5611(PCSTR devname, s32 devid) :
    device(devname, devid),
    _reports(NULL),
    _measure_phase(false),
    _TEMP(0),
    _OFF(0),
    _SENS(0),
    _msl_pressure(101325)

{
    _prom.factory_setup = 0;
	_prom.c1_pressure_sens = 0;
	_prom.c2_pressure_offset = 0;
	_prom.c3_temp_coeff_pres_sens = 0;
	_prom.c4_temp_coeff_pres_offset = 0;
	_prom.c5_reference_temp = 0;
	_prom.c6_temp_coeff_temp = 0;
	_prom.serial_and_crc = 0;

    _params.name = "ms5611_thread";
	_params.priority = 0;
	_params.stacksize = 256;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

ms5611::~ms5611(void)
{

}

s32 ms5611::probe(spi *pspi, gpio *gpio_cs)
{
    ASSERT((pspi != NULL) && (gpio_cs != NULL));

	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _devname);
		goto fail0;
	}

    _gpio_cs = gpio_cs;
    _spi = pspi;

	_gpio_cs->set_direction_output();
    _gpio_cs->set_value(true);

    ms5611::init();

    return 0;

fail0:
    return -1;
}

void ms5611::run(void *parg)
{
    for (;;) {
        measure();
        msleep(3);
    }
}

s32 ms5611::read(u8 *buf, u32 size)
{
    u32 count = size / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buf);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

    /*
     * While there is space in the caller's buffer, and reports, copy them.
     * Note that we may be pre-empted by the workq thread while we are doing this;
     * we are careful to avoid racing with them.
     */
    while (count--) {
        if (_reports->get(brp)) {
	        ret += sizeof(*brp);
            brp++;
        }
    }

    /* if there was no data, warn the caller */
    return ret ? ret : -EAGAIN;
}


s32 ms5611::init(void)
{
	ms5611::reset();
	ms5611::read_prom();
	/* allocate basic report buffers */
	_reports = new ringbuffer(2, sizeof(baro_report));
	if (_reports == NULL) {
		ERR("can't get memory for reports");
		//ret = -ENOMEM;
		goto fail0;
	}

    return 0;

fail0:
    return -1;

}

s32 ms5611::reset(void)
{
    ms5611::write_cmd8(ADDR_RESET_CMD);

    /*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
    core::mdelay(MS5611_CONVERSION_INTERVAL_MS);
}

//Maximum values for calculation results:
//PMIN = 10mbar PMAX = 1200mbar
//TMIN = -40°C TMAX = 85°C TREF = 20°C
void ms5611::measure(void)
{
	s32 ret;
	u32 raw;

	//perf_begin(_sample_perf);

	struct baro_report report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = 0;//hrt_absolute_time();
    report.error_count = 0;//perf_event_count(_comms_errors);

    union cvt {
        u8 byte[4];
        u32 raw;
    };

    u8 addr = 0;
    union cvt data;
    u8 buf[3] = { 0 };

    /* _measure_phase == 0 do temperature first, else now do a pressure measurement */
    addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

    taskENTER_CRITICAL();
    ms5611::write_cmd8(addr);
    taskEXIT_CRITICAL();

    msleep(MS5611_CONVERSION_INTERVAL_MS);

    /* read the most recent measurement */
    taskENTER_CRITICAL();
    ret = read_reg(ADDR_DATA, buf, sizeof(buf));
    taskEXIT_CRITICAL();
	if (ret < 0) {
		//perf_count(_comms_errors);
		//perf_end(_sample_perf);
		goto out;
	}

    /* fetch the raw value */
    data.byte[0] = buf[2];
    data.byte[1] = buf[1];
    data.byte[2] = buf[0];
    data.byte[3] = 0;

    raw = data.raw;

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* temperature offset (in ADC units) */ //实际和参考温度之间的差异
		s32 dT = (s32)raw - ((s32)_prom.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		_TEMP = 2000 + (s32)(((s64)dT * _prom.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */ //实际温度抵消、实际温度灵敏度
		_SENS = ((s64)_prom.c1_pressure_sens << 15) + (((s64)_prom.c3_temp_coeff_pres_sens * dT) >> 8);
		_OFF  = ((s64)_prom.c2_pressure_offset << 16) + (((s64)_prom.c4_temp_coeff_pres_offset * dT) >> 7);

		/* temperature compensation */
		if (_TEMP < 2000) {

			s32 T2 = POW2(dT) >> 31;

			s64 f = POW2((s64)_TEMP - 2000);
			s64 OFF2 = 5 * f >> 1;
			s64 SENS2 = 5 * f >> 2;

			if (_TEMP < -1500) {
				s64 f2 = POW2(_TEMP + 1500);
				OFF2 += 7 * f2;
				SENS2 += 11 * f2 >> 1;
			}

			_TEMP -= T2;
			_OFF  -= OFF2;
			_SENS -= SENS2;
		}

	} else {

		/* pressure calculation, result in Pa */
		s32 P = (((raw * _SENS) >> 21) - _OFF) >> 15;
		_P = P * 0.01f;
		_T = _TEMP * 0.01f;

		/* generate a new report */
		report.temperature = _TEMP / 100.0f;
		report.pressure = P / 100.0f;		/* convert to millibar */

		/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

		/*
		 * PERFORMANCE HINT:
		 *
		 * The single precision calculation is 50 microseconds faster than the double
		 * precision variant. It is however not obvious if double precision is required.
		 * Pending more inspection and tests, we'll leave the double precision variant active.
		 *
		 * Measurements:
		 * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
		 *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
		 */

		/* tropospheric properties (0-11km) for standard atmosphere */
		const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
		const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
		const double g  = 9.80665;	/* gravity constant in m/s/s */
		const double R  = 287.05;	/* ideal gas constant in J/kg/K */

		/* current pressure at MSL in kPa */
		double p1 = _msl_pressure / 1000.0;

		/* measured pressure in kPa */
		double p = P / 1000.0;

		/*
		 * Solve:
		 *
		 *     /        -(aR / g)     \
		 *    | (p / p1)          . T1 | - T1
		 *     \                      /
		 * h = -------------------------------  + h1
		 *                   a
		 */
		report.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
        if (_reports->force(&report)) {
			//perf_count(_buffer_overflows);
		}
    }

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5611_MEASUREMENT_RATIO + 1);

	//perf_end(_sample_perf);

	return;

out:
    return;

}




void ms5611::read_prom(void)
{
	/* read and convert PROM words */
    u8 buf[2] = { 0 };
    ms5611::read_reg(ADDR_PROM_SETUP, buf, 2);
    _prom.factory_setup = buf[0] << 8 | buf[1];
    ms5611::read_reg(ADDR_PROM_C1, buf, 2);
    _prom.c1_pressure_sens = buf[0] << 8 | buf[1];
    ms5611::read_reg(ADDR_PROM_C2, buf, 2);
    _prom.c2_pressure_offset = buf[0] << 8 | buf[1];
    ms5611::read_reg(ADDR_PROM_C3, buf, 2);
    _prom.c3_temp_coeff_pres_sens = buf[0] << 8 | buf[1];
    ms5611::read_reg(ADDR_PROM_C4, buf, 2);
    _prom.c4_temp_coeff_pres_offset = buf[0] << 8 | buf[1];
    ms5611::read_reg(ADDR_PROM_C5, buf, 2);
    _prom.c5_reference_temp = buf[0] << 8 | buf[1];
    ms5611::read_reg(ADDR_PROM_C6, buf, 2);
    _prom.c6_temp_coeff_temp = buf[0] << 8 | buf[1];
    ms5611::read_reg(ADDR_PROM_CRC, buf, 2);
    _prom.serial_and_crc = buf[0] << 8 | buf[1];

    u16 tmp = 0;
    ms5611::read_reg(ADDR_RESET_CMD, (u8 *)&tmp, 2);
	/* calculate CRC and return success/failure accordingly */
	s32 ret = ms5611::crc4((u16 *)&_prom);
    if (ret != 0) {
		ERR("crc failed");
    }

    //return ret;
}

/**
 * MS5611 crc4 cribbed from the datasheet
 */
s32 ms5611::crc4(u16 *prom)
{
	s16 cnt;
	u16 rem;
	u16 crc_read;
	u8 bit;

	rem = 0x00;
	/* save the read crc */
	crc_read = prom[7];
	/* remove CRC byte */
	prom[7] = (0xFF00 & (prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			rem ^= (u8)((prom[cnt >> 1]) & 0x00FF);
		} else {
			rem ^= (u8)(prom[cnt >> 1] >> 8);
		}

		for (bit = 8; bit > 0; bit--) {
			if (rem & 0x8000) {
				rem = (rem << 1) ^ 0x3000;
			} else {
				rem = (rem << 1);
			}
		}
	}
	/* final 4 bit remainder is CRC value */
	rem = (0x000F & (rem >> 12));
	prom[7] = crc_read;
	/* return true if CRCs match */
	if ((0x000F & crc_read) != (rem ^ 0x00)) {
        return -1;
    }

    return 0;
}

s32 ms5611::self_test(u8 cmd_osr)
{
    union cvt {
        u8 byte[4];
        u32 raw;
    };

    ms5611::write_cmd8(cmd_osr);

    union cvt data;
    u8 buf[3] = { 0 };
    /* read the most recent measurement */
    s32 ret = ms5611::read_reg(ADDR_DATA, buf, sizeof(buf));
    if (ret == 0) {
        /* fetch the raw value */
        data.byte[0] = buf[2];
        data.byte[1] = buf[1];
        data.byte[2] = buf[0];
        data.byte[3] = 0;

        return (data.raw); //(u32)((buf[0]<<16) | (buf[1]<<8) | (buf[2]));
    }

    return -1;
}



s32 ms5611::write_cmd8(u8 cmd)
{
    s32 ret = 0;
    //cmd = cmd & SPI_WRITE_CMD /*WRITE_CMD*/;
    _gpio_cs->set_value(false);
    struct spi_msg msgs = {
        .buf = &cmd,
        .len = sizeof(cmd),
        .flags = 0,
    };
    ret = _spi->transfer(&msgs);
    if (ret < 0) {
        ERR("failed to spi transfer.\n");
        goto fail0;
    }

    //core::mdelay(0);
    core::udelay(100);
    _gpio_cs->set_value(true);

    return 0;

fail0:
    _gpio_cs->set_value(true);
    return -1;
}


s32 ms5611::read_reg8(u8 reg)
{
    s32 ret = 0;
    u8 data = 0;
    ms5611::read_reg(reg, &data, 1);
    if (ret < 0) {
        return -1;
    }

    return ((s32)data);
}


s32 ms5611::write_reg8(u8 reg, u8 data)
{
    s32 ret = 0;
    ret = ms5611::write_reg(reg, &data, 1);
    if (ret < 0) {
        return -1;
    }

    return 0;
}

s32 ms5611::read_reg(u8 reg, u8 *buf, u8 len)
{
    s32 ret = 0;
    //reg = reg | READ_CMD/*SPI_READ_CMD*/ ;
    _gpio_cs->set_value(false);
    struct spi_msg msgs[] = {
        [0] = {
            .buf = &reg,
            .len = sizeof(reg),
            .flags = 0,
        },
        [1] = {
            .buf = buf,
            .len = len,
            .flags = SPI_M_RD,
        },
    };
    ret = _spi->transfer(&msgs[0]);
    if (ret < 0) {
        goto fail0;
    }
    ret = _spi->transfer(&msgs[1]);
    if (ret < 0) {
        goto fail1;
    }

    _gpio_cs->set_value(true);
    return 0;

fail1:
fail0:
    _gpio_cs->set_value(true);
    return -1;
}

s32 ms5611::write_reg(u8 reg, u8 *buf, u8 len)
{
    s32 ret = 0;
   //reg = reg & WRITE_CMD /*SPI_WRITE_CMD*/ ;
    _gpio_cs->set_value(false);
    struct spi_msg msgs[] = {
        [0] = {
            .buf = &reg,
            .len = sizeof(reg),
            .flags = 0,
        },
        [1] = {
            .buf = buf,
            .len = len,
            .flags = 0,
        },
    };
    ret = _spi->transfer(&msgs[0]);
    if (ret < 0) {
        goto fail0;
    }
    ret = _spi->transfer(&msgs[1]);
    if (ret < 0) {
        goto fail1;
    }
    //core::mdelay(10);
    _gpio_cs->set_value(true);

    return 0;

fail1:
fail0:
    _gpio_cs->set_value(true);
    return -1;
}

}
/***********************************************************************
** End of file
***********************************************************************/



