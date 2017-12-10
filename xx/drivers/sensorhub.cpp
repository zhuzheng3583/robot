/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/06/12
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "sensorhub.h"

#include "core.h"
#include "stdlib.h"

#include "usbd_customhid_if.h"

namespace driver {

sensorhub::sensorhub(PCSTR name, s32 id) :
	device(name, id)
{
	
}

sensorhub::~sensorhub(void)
{

}

s32 sensorhub::probe(usb_dev *pusb_dev)
{
	ASSERT(pusb_dev != NULL);
	
	_usb_dev = pusb_dev;

	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}
	
	if (_usb_dev->is_probed() < 0) {
		ERR("%s: failed to usb devices No probe.\n", _name);
		goto fail1;
	}

	_in_buf_size = (sizeof(sensorhub_event_t) / _usb_dev->_epin_size + 1) * _usb_dev->_epin_size;
	_in_buf_data = (sensorhub_event_t *)malloc(_in_buf_size);
	if (_in_buf_data == NULL) {
		ERR("%s: failed to sensorhub malloc.\n", _name);
		goto fail2;
	} 

	INF("%s: probe success.\n", _name);
	return 0;
	
fail2:
fail1:
	device::remove();
fail0:
	return -1;	
}

s32 sensorhub::remove(void)
{
	if (device::remove() < 0) {
		goto fail0;
	}
		
	if (_in_buf_data != NULL) {
		free(_in_buf_data);
	}

	return 0;

fail0:
    return -1;	
}

s32 sensorhub::read(u8 *buf, u32 size)
{
	u32 readcnt = 0;

	return readcnt;
}

s32 sensorhub::write(u8 *buf, u32 size)
{
    u32 writecnt = 0;

	return writecnt;
}
     
s32 sensorhub::read(void)
{
	_in_buf_data->acceleration.x = 1;
	_in_buf_data->acceleration.y = 2;
	_in_buf_data->acceleration.z = 3;

	_in_buf_data->magnetic.x = 1;
	_in_buf_data->magnetic.y = 2;
	_in_buf_data->magnetic.z = 3;

	_in_buf_data->orientation.azimuth = 1;
	_in_buf_data->orientation.pitch = 2;
	_in_buf_data->orientation.roll = 3;

	_in_buf_data->gyro.x = 1;
	_in_buf_data->gyro.y = 2;
	_in_buf_data->gyro.z = 3;

	_in_buf_data->distance = 2;
	_in_buf_data->light = 2;
	
	return 0;
}

s32 sensorhub::report(void)
{
	_usb_dev->write((u8 *)_in_buf_data, _in_buf_size);
	return 0;
}



s32 sensorhub::self_test(void)
{	
	sensorhub::open();
	while (1) {
		sensorhub::read();
		//core::mdelay(20);
		sensorhub::report();
	}
}

}

/***********************************************************************
** End of file
***********************************************************************/

