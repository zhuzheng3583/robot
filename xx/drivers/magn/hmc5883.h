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

//bit0-bit1 xyz是否使用偏压,默认为0正常配置
//bit2-bit4 数据输出速率, 110为最大75HZ 100为15HZ 最小000 0.75HZ
//bit5-bit5每次采样平均数 11为8次 00为一次
#define ADDR_CONFIG_A 	0x00 	// r/w

//bit7-bit5磁场增益 数据越大,增益越小 默认001
#define ADDR_CONFIG_B 	0x01 	// r/w

//bit0-bit1 模式设置 00为连续测量 01为单一测量
#define ADDR_MODE 		0x02 	// r/w
#define ADDR_DATA_OUT_X_MSB 	0x03	// r
#define ADDR_DATA_OUT_X_LSB 	0x04	// r
#define ADDR_DATA_OUT_Z_MSB 	0x05	// r
#define ADDR_DATA_OUT_Z_LSB 	0x06	// r
#define ADDR_DATA_OUT_Y_MSB 	0x07	// r
#define ADDR_DATA_OUT_Y_LSB 	0x08	// r

//bit1 数据更新时该位自动锁存,等待用户读取,读取到一半的时候防止数据改变
//bit0 数据已经准备好等待读取了,DRDY引脚也能用
#define ADDR_STATUS 		0x09	// r

//三个识别寄存器,用于检测芯片完整性
#define ADDR_ID_A	0x0A	// r
#define ADDR_ID_B	0x0B	// r
#define ADDR_ID_C	0x0C	// r

//三个识别寄存器的默认值
//0x48
#define ID_A_WHO_AM_I			'H'
//0x34
#define ID_B_WHO_AM_I			'4'
//0x33
#define ID_C_WHO_AM_I			'3'


#define HMC5883_SLAVE_ADDRESS 0x3C //写地址,读地址+1


//HMC5883 初始化宏定义
#define HMC_DEFAULT_CONFIGA_VALUE       0x78     //75hz 8倍采样 正常配置
#define HMC_DEFAULT_CONFIGB_VALUE       0x00     //+-0.88GA增益
#define HMC_DEFAULT_MODE_VALUE          0x00     //连续测量模式


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

