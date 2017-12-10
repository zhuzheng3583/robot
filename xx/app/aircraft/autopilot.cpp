/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/23
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "autopilot.h"

#include "leader_system.h"

namespace app {

autopilot::autopilot(void)
{
	_params.name = "autopilot";
	_params.priority = 0;
	_params.stacksize = 1024;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

autopilot::~autopilot(void)
{

}

void autopilot::run(void *parg)
{
    u32 cnt = 0;
    u8 led_on = 0;
    gpio *led_blue = leader_system::get_instance()->get_led_blue();
    niming *niming = leader_system::get_instance()->get_niming();

	mpu6000 *mpu6000 = leader_system::get_instance()->get_mpu6000();
	mpu6000->open();
	ms5611 *ms5611 = leader_system::get_instance()->get_ms5611();
	ms5611->open();
	hmc5883 *hmc5883 = leader_system::get_instance()->get_hmc5883();
	hmc5883->open();
	ashtech *ashtech = leader_system::get_instance()->get_ashtech();
	ashtech->open();
    sbus *sbus = leader_system::get_instance()->get_sbus();
	sbus->open();

	struct accel_report accel;
	memset(&accel, 0, sizeof(accel));
	struct gyro_report gyro;
	memset(&gyro, 0, sizeof(gyro));
    struct mag_report mag;
	memset(&mag, 0, sizeof(mag));
    struct baro_report baro;
	memset(&baro, 0, sizeof(baro));
    struct vehicle_gps_position_s gps_position;
    memset(&gps_position, 0, sizeof(gps_position));
	struct satellite_info_s satellite_info;
	memset(&satellite_info, 0, sizeof(satellite_info));   
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct rc_input_values rc_values;
    memset(&rc_values, 0, sizeof(rc_values));
    float euler[3] = { 0 };

    //mpu6000->calibrate_accel();
    mpu6000->calibrate_gyro();

	struct pollfd fds = {
		.events = POLLIN,
	};

	for (; ;)
	{
		//mpu6000->poll(&fds, 1000);
		mpu6000->read_accel((u8 *)&accel, sizeof(accel_report));
		mpu6000->read_gyro((u8 *)&gyro, sizeof(gyro_report));
		//DBG("%s: accel.x_raw=%d, accel.y_raw=%d, accel.z_raw=%d, "
		//	"gyro.x_raw=%d, gyro.y_raw=%d, gyro.z_raw=%d.\n",
		//	_os_name, accel.x_raw, accel.y_raw, accel.z_raw,
		//	gyro.x_raw, gyro.y_raw, gyro.z_raw);

		hmc5883->read((u8 *)&mag, sizeof(mag_report));
		//DBG("%s: mag.x_raw=%d, mag.y_raw=%d, mag.z_raw=%d.\n",
		//	_os_name, mag.x_raw, mag.y_raw, mag.z_raw);

		ms5611->read((u8 *)&baro, sizeof(baro_report));
		//DBG("%s: temperature=%f, pressure=%f, altitude=%f.\n",
		//	_os_name, baro.temperature, baro.pressure, baro.altitude);
        
        ashtech->read_gps_position((u8 *)&gps_position, sizeof(gps_position));
        ashtech->read_satellite_info((u8 *)&satellite_info, sizeof(satellite_info));
        //DBG("%s: lat=%d, lon=%d, alt=%d.\n",
		//	_os_name, gps_position.lat, gps_position.lon, gps_position.alt);
        
        sbus->read((u8 *)&rc_values, sizeof(rc_values));
        DBG("%s: rc_values:\n", _os_name);
        for (u8 i = 0; i < SBUS_INPUT_CHANNELS; i++) {
            DBG("val[%d]=%d ", i, rc_values.values[i]);
        }
        DBG("\n%s: rc_values end.\n", _os_name);
        
		imu_update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, &att);
		niming->report_status(&att);
        niming->report_sensor(false, &accel, &gyro, &mag);

        led_blue->set_value((led_on = !led_on) ? VHIGH : VLOW);
        //msleep(1);
        u32 stack_size = uxTaskGetStackHighWaterMark((TaskHandle_t)_os_handle);
		//DBG("%s: task is active[%u], left stack size = %ubyte\n", _os_name, cnt++, stack_size);
    }
}

}
/***********************************************************************
** End of file
***********************************************************************/


