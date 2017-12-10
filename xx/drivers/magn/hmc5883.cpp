/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/31
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "hmc5883.h"
#include <math.h>
#include "calibration_routines.h"

namespace driver {

hmc5883::hmc5883(PCSTR devname, s32 devid) :
    device(devname, devid),
    _mag_reports(NULL),
    _range_scale(0), /* default range scale from counts to gauss */
    _range_ga(1.3f),
    _range_bits(0)
{
	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	_params.name = "hmc5883_thread";
	_params.priority = 0;
	_params.stacksize = 256;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

hmc5883::~hmc5883(void)
{

}

s32 hmc5883::probe(i2c *pi2c, u8 slave_addr)
{
    ASSERT(pi2c != NULL);

    _i2c = pi2c;
    _slave_addr = slave_addr;

    s32 ret = init();
    if (ret < 0) {
		ERR("%s: failed to init.\n", _devname);
        goto fail0;
    }

    return 0;

fail0:
    return -1;
}



s32 hmc5883::open(void)
{
    return 0;
}

s32 hmc5883::read(u8 *buf, u32 size)
{
    u32 count = size / sizeof(struct mag_report);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	//if (_call_interval == 0) {
	//	_accel_reports->flush();
	//	measure();
	//}

	/* if no data, error (we could block here) */
	if (_mag_reports->empty())
		return -EAGAIN;

	//perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
    struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buf);
	int transferred = 0;
	while (count--) {
		if (!_mag_reports->get(mag_buf))
			break;
		transferred++;
		mag_buf++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(mag_report));

}

void hmc5883::run(void *parg)
{
    for (;;) {
        /* Send the command to begin a measurement. */
        write_reg8(ADDR_MODE, MODE_REG_SINGLE_MODE);

        /* wait for it to complete */
        msleep(HMC5883_CONVERSION_INTERVAL_MS);

        measure();
    }
}


s32 hmc5883::init(void)
{
	/* allocate basic report buffers */
	_mag_reports = new ringbuffer(2, sizeof(mag_report));
	if (_mag_reports == NULL)
		goto fail0;

	u8 id_a = hmc5883::read_reg8(ADDR_ID_A);
	u8 id_b = hmc5883::read_reg8(ADDR_ID_B);
	u8 id_c = hmc5883::read_reg8(ADDR_ID_C);
    if ((id_a != ID_A_WHO_AM_I) || (id_b != ID_B_WHO_AM_I) || (id_c != ID_C_WHO_AM_I)) {
        return -1;
    }

    hmc5883::write_reg8(ADDR_CONFIG_A, HMC_DEFAULT_CONFIGA_VALUE);
    hmc5883::write_reg8(ADDR_CONFIG_B, HMC_DEFAULT_CONFIGB_VALUE);
    hmc5883::write_reg8(ADDR_MODE, HMC_DEFAULT_MODE_VALUE); //初始化HMC5883


    hmc5883::reset();

    return 0;

fail0:
    return -1;
}

s32 hmc5883::reset(void)
{
	/* set range */
	return set_range(_range_ga);
}

s32 hmc5883::set_range(u32 range)
{
	if (range < 1) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 1370.0f;
		_range_ga = 0.88f;

	} else if (range <= 1) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 1090.0f;
		_range_ga = 1.3f;

	} else if (range <= 2) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 820.0f;
		_range_ga = 1.9f;

	} else if (range <= 3) {
		_range_bits = 0x03;
		_range_scale = 1.0f / 660.0f;
		_range_ga = 2.5f;

	} else if (range <= 4) {
		_range_bits = 0x04;
		_range_scale = 1.0f / 440.0f;
		_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		_range_bits = 0x05;
		_range_scale = 1.0f / 390.0f;
		_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		_range_bits = 0x06;
		_range_scale = 1.0f / 330.0f;
		_range_ga = 5.6f;

	} else {
		_range_bits = 0x07;
		_range_scale = 1.0f / 230.0f;
		_range_ga = 8.1f;
	}

	int ret;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg8(ADDR_CONFIG_B, (_range_bits << 5));

	//if (0 != ret)
		//perf_count(_comms_errors);

	u8 range_bits_in = 0;
	range_bits_in = read_reg8(ADDR_CONFIG_B);

	//if (0 != ret)
	//	perf_count(_comms_errors);

	return !(range_bits_in == (_range_bits << 5));
}


void hmc5883::measure(void)
{
#pragma pack(push, 1)
    struct { /* status register and data as read back from the device */
        u8  x[2];
        u8  z[2];
        u8  y[2];
    } hmc_report;
#pragma pack(pop)
    struct {
        s16 x;
        s16 y;
        s16 z;
    } report;

    s32 ret;
    u8 check_counter;

    //perf_begin(_sample_perf);
    struct mag_report new_report;
    bool sensor_is_onboard = false;

    float xraw_f;
    float yraw_f;
    float zraw_f;

    /* this should be fairly close to the end of the measurement, so the best approximation of the time */
    new_report.timestamp = 0;//hrt_absolute_time();
    new_report.error_count = 0;//perf_event_count(_comms_errors);

    /*
     * @note  We could read the status register here, which could tell us that
     *        we were too early and that the output registers are still being
     *        written.  In the common case that would just slow us down, and
     *        we're better off just never being early.
     */

    /* get measurements from the device */
    ret = read_reg(ADDR_DATA_OUT_X_MSB, (u8 *)&hmc_report, sizeof(hmc_report));
    if (ret < 0) {
        //perf_count(_comms_errors);
        //debug("data/status read error");
        goto out;
    }

    /* swap the data we just received */
    report.x = (((s16)hmc_report.x[0]) << 8) + hmc_report.x[1];
    report.y = (((s16)hmc_report.y[0]) << 8) + hmc_report.y[1];
    report.z = (((s16)hmc_report.z[0]) << 8) + hmc_report.z[1];

    /*
     * If any of the values are -4096, there was an internal math error in the sensor.
     * Generalise this to a simple range check that will also catch some bit errors.
     */
    if ((abs(report.x) > 2048) ||
        (abs(report.y) > 2048) ||
        (abs(report.z) > 2048)) {
        //perf_count(_comms_errors);
        goto out;
    }

    /* get measurements from the device */
    new_report.temperature = 0;

    /*
     * RAW outputs
     *
     * to align the sensor axes with the board, x and y need to be flipped
     * and y needs to be negated
     */
    new_report.x_raw = report.y;
    new_report.y_raw = -report.x;
    /* z remains z */
    new_report.z_raw = report.z;

    /* scale values for output */


    /* the standard external mag by 3DR has x pointing to the
     * right, y pointing backwards, and z down, therefore switch x
     * and y and invert y
     */
    xraw_f = -report.y;
    yraw_f = report.x;
    zraw_f = report.z;

    // apply user specified rotation
    //rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
    /* flip axes and negate value for y */
    new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
    /* z remains z */
    new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;


    _last_report = new_report;

    /* post a report to the ring */
    if (_mag_reports->force(&new_report)) {
        //perf_count(_buffer_overflows);
    }

    ret = 0;

out:
    //perf_end(_sample_perf);
    return;
}


s32 hmc5883::calibrate_mag(void)
{
	INF(CAL_STARTED_MSG, "mag_hmc5883");
	sleep(1);

	struct mag_scale mag_scale = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	s32 ret = 0;
	u32 calibrated_ok = 0;

	INF("Calibrating magnetometer..");
	
	memcpy(&_scale, &mag_scale, sizeof(mag_scale));

#if 0
	if (ret == 0) {
		/* calibrate range */
		res = ioctl(fd, MAGIOCCALIBRATE, fd);

		if (res != OK) {
			mavlink_and_console_log_info(mavlink_fd, "Skipped scale calibration");
			/* this is non-fatal - mark it accordingly */
			res = OK;
		}
	}
#endif

	if (ret == 0) {
		ret = calibrate_instance();

		if (ret == 0) {
			calibrated_ok++;
		}
	}

	if (calibrated_ok) {
		INF(CAL_DONE_MSG, "mag_hmc5883");

		/* auto-save to EEPROM */
		//res = param_save_default();

		if (ret != 0) {
			ERR(CAL_FAILED_SAVE_PARAMS_MSG);
		}
	} else {
		ERR(CAL_FAILED_MSG, "mag_hmc5883");
	}

	return ret;
}

s32 hmc5883::calibrate_instance(void)
{
	/* 45 seconds */
	u64 calibration_interval = 45 * 1000 * 1000;

	/* maximum 500 values */
	const s32 calibration_maxcount = 240;
	u32 calibration_counter;

	float *x = NULL;
	float *y = NULL;
	float *z = NULL;

	s32 ret = 0;
	
	/* allocate memory */
	INF(CAL_PROGRESS_MSG, "mag_hmc5883", 20);

	x = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_maxcount));
	y = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_maxcount));
	z = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_maxcount));

	if (x == NULL || y == NULL || z == NULL) {
		ERR("ERROR: out of memory");

		/* clean up */
		if (x != NULL) {
			free(x);
		}

		if (y != NULL) {
			free(y);
		}

		if (z != NULL) {
			free(z);
		}

		ret = ERROR;
		return ret;
	}

	if (ret == 0) {
		struct mag_report mag_report;

		/* limit update rate to get equally spaced measurements over time (in ms) */
		//orb_set_interval(sub_mag, (calibration_interval / 1000) / calibration_maxcount);

		/* calibrate offsets */
		//uint64_t calibration_deadline = hrt_absolute_time() + calibration_interval;
		u32 poll_errcount = 0;

		INF("Turn on all sides: front/back,left/right,up/down");

		calibration_counter = 0U;
		s32 size = 0;
		while (calibration_counter < calibration_maxcount) {
			/* wait blocking for new data */
			/* now go get it */
			size = hmc5883::read((u8 *)&mag_report, sizeof(mag_report));
			if (size != sizeof(mag_report)) {
				poll_errcount++;
		       	//ERR("%s: ERROR: READ 1.\n", _devname);
		      	core::mdelay(2);
		      	continue;
			}

			x[calibration_counter] = mag_report.x;
			y[calibration_counter] = mag_report.y;
			z[calibration_counter] = mag_report.z;
			calibration_counter++;

			if (calibration_counter % (calibration_maxcount / 20) == 0) {
				INF(CAL_PROGRESS_MSG, "mag_hmc5883", 20 + (calibration_counter * 50) / calibration_maxcount);
			}
            
            if (poll_errcount > 1000) {
                ERR(CAL_FAILED_SENSOR_MSG);
                ret = ERROR;
                break;
            }
		}
	}

	float sphere_x;
	float sphere_y;
	float sphere_z;
	float sphere_radius;

	if (ret == 0 && calibration_counter > (calibration_maxcount / 2)) {
		/* sphere fit */
		INF(CAL_PROGRESS_MSG, "mag_hmc5883", 70);
		sphere_fit_least_squares(x, y, z, calibration_counter, 100, 0.0f, &sphere_x, &sphere_y, &sphere_z, &sphere_radius);
		INF(CAL_PROGRESS_MSG, "mag_hmc5883", 80);

		if (!isfinite(sphere_x) || !isfinite(sphere_y) || !isfinite(sphere_z)) {
			ERR("ERROR: NaN in sphere fit");
			ret = ERROR;
		}
	}

	if (x != NULL) {
		free(x);
	}

	if (y != NULL) {
		free(y);
	}

	if (z != NULL) {
		free(z);
	}

    struct mag_scale mag_scale;
	if (ret == 0) {
		/* apply calibration and set parameters */
		mag_scale.x_offset = sphere_x;
		mag_scale.y_offset = sphere_y;
		mag_scale.z_offset = sphere_z;
		memcpy(&_scale, &mag_scale, sizeof(mag_scale));
	}

#if 0
		if (ret == 0) {

			bool failed = false;
			/* set parameters */
			(void)sprintf(str, "CAL_MAG%u_ID", s);
			failed |= (OK != param_set(param_find(str), &(device_id)));
			(void)sprintf(str, "CAL_MAG%u_XOFF", s);
			failed |= (OK != param_set(param_find(str), &(mscale.x_offset)));
			(void)sprintf(str, "CAL_MAG%u_YOFF", s);
			failed |= (OK != param_set(param_find(str), &(mscale.y_offset)));
			(void)sprintf(str, "CAL_MAG%u_ZOFF", s);
			failed |= (OK != param_set(param_find(str), &(mscale.z_offset)));
			(void)sprintf(str, "CAL_MAG%u_XSCALE", s);
			failed |= (OK != param_set(param_find(str), &(mscale.x_scale)));
			(void)sprintf(str, "CAL_MAG%u_YSCALE", s);
			failed |= (OK != param_set(param_find(str), &(mscale.y_scale)));
			(void)sprintf(str, "CAL_MAG%u_ZSCALE", s);
			failed |= (OK != param_set(param_find(str), &(mscale.z_scale)));

			if (failed) {
				res = ERROR;
				ERR(CAL_FAILED_SET_PARAMS_MSG);
			}

			INF(CAL_PROGRESS_MSG, sensor_name, 90);
		}
#endif
    INF("mag off: x:%.2f y:%.2f z:%.2f Ga.\n", (double)mag_scale.x_offset, (double)mag_scale.y_offset, (double)mag_scale.z_offset);
    INF("mag scale: x:%.2f y:%.2f z:%.2f.\n", (double)mag_scale.x_scale, (double)mag_scale.y_scale, (double)mag_scale.z_scale);

	return ret;
}


s32 hmc5883::self_test(void)
{
	f64 angle;
	u32 acr;
    //连续读出HMC5883内部角度数据，地址范围0x3~0x5
    u8 tmp[6];
    s16 data[3];
    if (read_reg(ADDR_DATA_OUT_X_MSB, tmp, 6))
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1];//Combine MSB and LSB of X Data output register
    data[1] = (tmp[2] << 8) | tmp[3];//Combine MSB and LSB of Z Data output register
    data[2] = (tmp[4] << 8) | tmp[5];//Combine MSB and LSB of Y Data output register
    angle = atan2((f64)data[2],(f64)data[0])*(180/3.14159265)+180;//单位：角度 (0~360)
    angle *= 10;
    acr = (u32)angle;
    INF("%s: %f, %d.\n", _devname, angle, acr);

    return 0;
}




s32 hmc5883::read_reg8(u8 reg)
{
    s32 ret = 0;
    u8 data = 0;
    hmc5883::read_reg(reg, &data, 1);
    if (ret < 0) {
        return -1;
    }

    return ((s32)data);
}


s32 hmc5883::write_reg8(u8 reg, u8 data)
{
    s32 ret = 0;
    ret = hmc5883::write_reg(reg, &data, 1);
    if (ret < 0) {
        return -1;
    }

    return 0;
}

s32 hmc5883::read_reg(u8 reg, u8 *buf, u8 len)
{
	struct i2c_msg msgs[] = {
		[0] = {
			.addr = _slave_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		[1] = {
			.addr = _slave_addr + 1,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	if (_i2c->transfer(msgs, 2) != 0) {
		ERR("%s: failed to i2c transfer!\n", _devname);
        return -1;
	}

    return 0;
}

s32 hmc5883::write_reg(u8 reg, u8 *buf, u8 len)
{
	struct i2c_msg msgs[] = {
		[0] = {
			.addr = _slave_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		[1] = {
			.addr = _slave_addr,
			.flags = 0,
			.len = len,
			.buf = buf,
		},
	};
	if (_i2c->transfer(msgs, 2) != 0) {
		ERR("%s: failed to i2c transfer!\n", _devname);
        return -1;
	}

    return 0;
}

}
/***********************************************************************
** End of file
***********************************************************************/

