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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "device.h"
#include "dma.h"
#include "i2c.h"

#include "ringbuffer.h"
#include "magn.h"

#include "os/thread.h"
#include "calibration_messages.h"

//bit0-bit1 xyz�Ƿ�ʹ��ƫѹ,Ĭ��Ϊ0��������
//bit2-bit4 �����������, 110Ϊ���75HZ 100Ϊ15HZ ��С000 0.75HZ
//bit5-bit5ÿ�β���ƽ���� 11Ϊ8�� 00Ϊһ��
#define ADDR_CONFIG_A 	0x00 	// r/w

//bit7-bit5�ų����� ����Խ��,����ԽС Ĭ��001
#define ADDR_CONFIG_B 	0x01 	// r/w

//bit0-bit1 ģʽ���� 00Ϊ�������� 01Ϊ��һ����
#define ADDR_MODE 		0x02 	// r/w
#define ADDR_DATA_OUT_X_MSB 	0x03	// r
#define ADDR_DATA_OUT_X_LSB 	0x04	// r
#define ADDR_DATA_OUT_Z_MSB 	0x05	// r
#define ADDR_DATA_OUT_Z_LSB 	0x06	// r
#define ADDR_DATA_OUT_Y_MSB 	0x07	// r
#define ADDR_DATA_OUT_Y_LSB 	0x08	// r

//bit1 ���ݸ���ʱ��λ�Զ�����,�ȴ��û���ȡ,��ȡ��һ���ʱ���ֹ���ݸı�
//bit0 �����Ѿ�׼���õȴ���ȡ��,DRDY����Ҳ����
#define ADDR_STATUS 		0x09	// r

//����ʶ��Ĵ���,���ڼ��оƬ������
#define ADDR_ID_A	0x0A	// r
#define ADDR_ID_B	0x0B	// r
#define ADDR_ID_C	0x0C	// r

//����ʶ��Ĵ�����Ĭ��ֵ
//0x48
#define ID_A_WHO_AM_I			'H'
//0x34
#define ID_B_WHO_AM_I			'4'
//0x33
#define ID_C_WHO_AM_I			'3'


#define HMC5883_SLAVE_ADDRESS 0x3C //д��ַ,����ַ+1


//HMC5883 ��ʼ���궨��
#define HMC_DEFAULT_CONFIGA_VALUE       0x78     //75hz 8������ ��������
#define HMC_DEFAULT_CONFIGB_VALUE       0x00     //+-0.88GA����
#define HMC_DEFAULT_MODE_VALUE          0x00     //��������ģʽ


/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150 */
#define HMC5883_CONVERSION_INTERVAL	(1000000 / 150)	/* microseconds */
#define HMC5883_CONVERSION_INTERVAL_MS	(6)


/* temperature on hmc5983 only */
#define ADDR_TEMP_OUT_MSB		0x31
#define ADDR_TEMP_OUT_LSB		0x32

/* modes not changeable outside of driver */
#define HMC5883L_MODE_NORMAL		(0 << 0)  /* default */
#define HMC5883L_MODE_POSITIVE_BIAS	(1 << 0)  /* positive bias */
#define HMC5883L_MODE_NEGATIVE_BIAS	(1 << 1)  /* negative bias */

#define HMC5883L_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5883L_AVERAGING_2		(1 << 5)
#define HMC5883L_AVERAGING_4		(2 << 5)
#define HMC5883L_AVERAGING_8		(3 << 5)

#define MODE_REG_CONTINOUS_MODE		(0 << 0)
#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

#define STATUS_REG_DATA_OUT_LOCK	(1 << 1) /* page 16: set if data is only partially read, read device to reset */
#define STATUS_REG_DATA_READY		(1 << 0) /* page 16: set if all axes have valid measurements */

#define HMC5983_TEMP_SENSOR_ENABLE	(1 << 7)

using namespace os;

namespace driver {

class hmc5883 : public device, public thread
{
public:
    hmc5883(PCSTR devname, s32 devid);
    ~hmc5883(void);

public:
	u8 _slave_addr;
	i2c *_i2c;
	unsigned		_measure_ticks;
	ringbuffer		*_mag_reports;
	mag_scale		_scale;
	float 			_range_scale;
	float 			_range_ga;
	u8				_range_bits;
	struct mag_report	_last_report;           /**< used for info() */

public:
    s32 probe(i2c *pi2c, u8 slave_addr);
    s32 remove(void);

public:
	virtual s32 open(void);
	virtual s32 read(u8 *buf, u32 size);
	virtual s32 close(void);
public:
	virtual void run(void *parg);


public:
    s32 init(void);
    s32 reset(void);
    void measure(void);
	s32 set_range(u32 range);

    s32 calibrate_instance(void);
    s32 calibrate_mag(void);
    
public:
	s32 self_test(void);

private:
	s32 read_reg8(u8 reg);
	s32 write_reg8(u8 reg, u8 data);
	s32 read_reg(u8 reg, u8 *buf, u8 len);
	s32 write_reg(u8 reg, u8 *buf, u8 len);
};

}
/***********************************************************************
** End of file
***********************************************************************/

