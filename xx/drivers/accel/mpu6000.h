/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
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
#include "spi.h"

#include "ringbuffer.h"
#include "accel.h"
#include "gyro.h"

#include "os/thread.h"
#include "calibration_messages.h"

#include "mathlib.h"
#include "lowpassfilter2p.h"
#include "rotation.h"

#define MPUREG_SAMPLE_RATE_DIV     0x19
#define MPUREG_CONFIG              0x1A

#define MPUREG_GYRO_CONFIG         0x1B
#define BITS_SELF_TEST_EN       0xE0
#define GYRO_CONFIG_FSR_SHIFT   3

#define MPUREG_ACCEL_CONFIG        0x1C
#define MPUREG_ACCEL_MOT_THR       0x1F
#define MPUREG_ACCEL_MOT_DUR       0x20
#define ACCL_CONFIG_FSR_SHIFT   3

#define MPUREG_ACCELMOT_THR        0x1F
#define MPUREG_ACCEL_MOT_DUR       0x20

#define MPUREG_FIFO_EN             0x23
#define FIFO_DISABLE_ALL        0x00
#define BIT_ACCEL_OUT           0x08
#define BITS_GYRO_OUT           0x70

#define MPUREG_INT_PIN_CFG         0x37
#define BIT_INT_ACTIVE_LOW      0x80
#define BIT_INT_OPEN_DRAIN      0x40
#define BIT_INT_LATCH_EN        0x20
#define BIT_INT_RD_CLR          0x10
#define BIT_I2C_BYPASS_EN       0x02
#define BIT_INT_CFG_DEFAULT     (BIT_INT_LATCH_EN | BIT_INT_RD_CLR)

#define MPUREG_INT_ENABLE          0x38
#define BIT_DATA_RDY_EN         0x01
#define BIT_DMP_INT_EN          0x02
#define BIT_FIFO_OVERFLOW       0x10
#define BIT_ZMOT_EN             0x20
#define BIT_MOT_EN              0x40
#define BIT_6500_WOM_EN         0x40

#define MPUREG_DMP_INT_STATUS      0x39

#define MPUREG_INT_STATUS          0x3A
#define BIT_DATA_RDY_INT        0x01
#define BIT_DMP_INT_INT         0x02
#define BIT_FIFO_OVERFLOW       0x10
#define BIT_ZMOT_INT            0x20
#define BIT_MOT_INT             0x40
#define BIT_6500_WOM_INT        0x40

#define MPUREG_RAW_ACCEL           0x3B
#define MPUREG_TEMPERATURE         0x41
#define MPUREG_RAW_GYRO            0x43
#define MPUREG_EXT_SENS_DATA_00    0x49

#define BIT_FIFO_RST            0x04
#define BIT_DMP_RST             0x08
#define BIT_I2C_MST_EN          0x20
#define BIT_FIFO_EN             0x40
#define BIT_DMP_EN              0x80
#define BIT_ACCEL_FIFO          0x08
#define BIT_GYRO_FIFO           0x70

#define MPUREG_DETECT_CTRL         0x69
#define MOT_DET_DELAY_SHIFT     4

#define MPUREG_USER_CTRL           0x6A
#define BIT_FIFO_EN             0x40
#define BIT_FIFO_RESET          0x04

#define MPUREG_PWR_MGMT_1          0x6B
#define BIT_H_RESET             0x80
#define BIT_SLEEP               0x40
#define BIT_CYCLE               0x20
#define BIT_CLK_MASK            0x07
#define BIT_RESET_ALL           0xCF
#define BIT_WAKEUP_AFTER_RESET  0x00

#define MPUREG_PWR_MGMT_2          0x6C
#define BIT_PWR_ACCEL_STBY_MASK 0x38
#define BIT_PWR_GYRO_STBY_MASK  0x07
#define BIT_LPA_FREQ_MASK       0xC0
#define BITS_PWR_ALL_AXIS_STBY  (BIT_PWR_ACCEL_STBY_MASK | \
                                    BIT_PWR_GYRO_STBY_MASK)

#define MPUREG_FIFO_COUNT_H        0x72
#define MPUREG_FIFO_R_W            0x74
#define MPUREG_WHOAMI              0x75

#define SAMPLE_DIV_MAX          0xFF
#define ODR_DLPF_DIS            8000
#define ODR_DLPF_ENA            1000

/* Min delay = MSEC_PER_SEC/ODR_DLPF_ENA */
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_ENA/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_DLPF   1
#define DELAY_MS_MAX_DLPF   256

/* Min delay = MSEC_PER_SEC/ODR_DLPF_DIS and round up to 1*/
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_DIS/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_NODLPF 1
#define DELAY_MS_MAX_NODLPF 32

/* device bootup time in millisecond */
#define POWER_UP_TIME_MS    100
/* delay to wait gyro engine stable in millisecond */
#define SENSOR_UP_TIME_MS   30
/* delay between power operation in millisecond */
#define POWER_EN_DELAY_US   10

#define MPU6050_LPA_5HZ     0x40

/* initial configure */
#define INIT_FIFO_RATE      200

#define DEFAULT_MOT_THR		1
#define DEFAULT_MOT_DET_DUR	1
#define DEFAULT_MOT_DET_DELAY	0

/* chip reset wait */
#define MPU6050_RESET_RETRY_CNT	10
#define MPU6050_RESET_WAIT_MS	20

/* FIFO related constant */
#define MPU6050_FIFO_SIZE_BYTE	1024
#define	MPU6050_FIFO_CNT_SIZE	2

enum mpu_device_id {
	MPU6050_ID = 0x68,
	MPU6500_ID = 0x70,
};

enum mpu_gyro_fsr {
	GYRO_FSR_250DPS = 0,
	GYRO_FSR_500DPS,
	GYRO_FSR_1000DPS,
	GYRO_FSR_2000DPS,
	NUM_GYRO_FSR
};

enum mpu_accl_fsr {
	ACCEL_FSR_2G = 0,
	ACCEL_FSR_4G,
	ACCEL_FSR_8G,
	ACCEL_FSR_16G,
	NUM_ACCL_FSR
};


enum mpu_filter {
	MPU_DLPF_256HZ_NOLPF2 = 0,
	MPU_DLPF_188HZ,
	MPU_DLPF_98HZ,
	MPU_DLPF_42HZ,
	MPU_DLPF_20HZ,
	MPU_DLPF_10HZ,
	MPU_DLPF_5HZ,
	MPU_DLPF_RESERVED,
	NUM_FILTER
};

enum mpu_clock_source {
	MPU_CLK_INTERNAL = 0,
	MPU_CLK_PLL_X,
	NUM_CLK
};



/* Sensitivity Scale Factor */
/* Sensor HAL will take 1024 LSB/g */
enum mpu_accel_fs_shift {
	ACCEL_SCALE_SHIFT_02G = 0,
	ACCEL_SCALE_SHIFT_04G = 1,
	ACCEL_SCALE_SHIFT_08G = 2,
	ACCEL_SCALE_SHIFT_16G = 3
};

enum mpu_gyro_fs_shift {
	GYRO_SCALE_SHIFT_FS0 = 3,
	GYRO_SCALE_SHIFT_FS1 = 2,
	GYRO_SCALE_SHIFT_FS2 = 1,
	GYRO_SCALE_SHIFT_FS3 = 0
};


// MPU 6000 registers
#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_TRIM1			0x0D    // ×Ô¼ì¼Ä´æÆ÷X
#define MPUREG_TRIM2			0x0E    // ×Ô¼ì¼Ä´æÆ÷Y
#define MPUREG_TRIM3			0x0F    // ×Ô¼ì¼Ä´æÆ÷Z
#define MPUREG_TRIM4			0x10    // ×Ô¼ì¼Ä´æÆ÷A
#define MPUREG_SMPLRT_DIV		0x19    // ²ÉÑùÆµÂÊ·ÖÆµÆ÷
#define MPUREG_CONFIG			0x1A    // ÅäÖÃ¼Ä´æÆ÷
#define MPUREG_GYRO_CONFIG		0x1B    // ÍÓÂÝÒÇÅäÖÃ¼Ä´æÆ÷
#define MPUREG_ACCEL_CONFIG	0x1C    // ¼ÓËÙ¶È¼ÆÅäÖÃ¼Ä´æÆ÷
#define MPUREG_ACCEL_MOT_THR	0X1F    // ÔË¶¯¼ì²â·§ÖµÉèÖÃ¼Ä´æÆ÷
#define MPUREG_ACCEL_MOT_DUR	0X20
#define MPUREG_INT_PIN_CFG		0x37    // ÖÐ¶Ï/ÅÔÂ·ÉèÖÃ¼Ä´æÆ÷
#define MPUREG_INT_ENABLE		0x38    // ÖÐ¶ÏÊ¹ÄÜ¼Ä´æÆ÷
#define MPUREG_INT_STATUS		0x3A    // ÖÐ¶Ï×´Ì¬¼Ä´æÆ÷
#define MPUREG_RAW_ACCEL		0X3B
#define MPUREG_ACCEL_XOUT_H	0x3B
#define MPUREG_ACCEL_XOUT_L	0x3C
#define MPUREG_ACCEL_YOUT_H	0x3D
#define MPUREG_ACCEL_YOUT_L	0x3E
#define MPUREG_ACCEL_ZOUT_H	0x3F
#define MPUREG_ACCEL_ZOUT_L	0x40
#define MPUREG_TEMPERATURE		0X41
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_RAW_GYRO		0X43
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_SIGPATH_RST		0X68    // ÐÅºÅÍ¨µÀ¸´Î»¼Ä´æÆ÷
#define MPUREG_MDETECT_CTRL	0X69    // ÔË¶¯¼ì²â¿ØÖÆ¼Ä´æÆ÷
#define MPUREG_USER_CTRL		0x6A    // ÓÃ»§¿ØÖÆ¼Ä´æÆ÷
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_EN			0x23    // FIFOÊ¹ÄÜ¼Ä´æÆ÷
#define MPUREG_FIFO_COUNTH		0x72    // FIFO¼ÆÊý¼Ä´æÆ÷¸ß°ËÎ»
#define MPUREG_FIFO_COUNTL		0x73    // FIFO¼ÆÊý¼Ä´æÆ÷µÍ°ËÎ»
#define MPUREG_FIFO_R_W		0x74    // FIFO¶ÁÐ´¼Ä´æÆ÷
#define MPUREG_WHOAMI			0x75    // Æ÷¼þID¼Ä´æÆ÷


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP               0x40
#define BIT_H_RESET			0x80
#define BITS_CLKSEL			0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS		0x10
#define BITS_FS_2000DPS		0x18
#define BITS_FS_MASK			0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2	0x00
#define BITS_DLPF_CFG_188HZ		0x01
#define BITS_DLPF_CFG_98HZ		0x02
#define BITS_DLPF_CFG_42HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF	0x07
#define BITS_DLPF_CFG_MASK		0x07
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01

// Product ID Description for MPU6000
// high 4 bits 	low 4 bits
// Product Name	Product Revision
#define MPU6000ES_REV_C4		0x14
#define MPU6000ES_REV_C5		0x15
#define MPU6000ES_REV_D6		0x16
#define MPU6000ES_REV_D7		0x17
#define MPU6000ES_REV_D8		0x18
#define MPU6000_REV_C4			0x54
#define MPU6000_REV_C5			0x55
#define MPU6000_REV_D6			0x56
#define MPU6000_REV_D7			0x57
#define MPU6000_REV_D8			0x58
#define MPU6000_REV_D9			0x59
#define MPU6000_REV_D10		0x5A

#define MPU6000_ACCEL_DEFAULT_RANGE_G			8
#define MPU6000_ACCEL_DEFAULT_RATE			    1000
#define MPU6000_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define MPU6000_GYRO_DEFAULT_RANGE_G			8
#define MPU6000_GYRO_DEFAULT_RATE			    1000
#define MPU6000_GYRO_DEFAULT_DRIVER_FILTER_FREQ  30

#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ		42

#define MPU6000_ONE_G					9.80665f

#define MPU_OUT_HZ  			200			// 100Hz [4, 1k]Hz DMP¹Ì¶¨Îª200Hz
#define MPU_DATA_MAX			0x7FFF		// 32767
#define MPU_DATA_MIN			0x8000		// -32768

#define q30                     1073741824.0f   //q30¸ñÊ½,long×ªfloatÊ±µÄ³ýÊý.


using namespace os;
using namespace math;

namespace driver {

class mpu6000 : public device, public thread
{
public:
	mpu6000(PCSTR devname, s32 devid);
	~mpu6000(void);

protected:
	u32	    _chip_select;
	spi	    *_spi;
	gpio    *_gpio_cs;

	ringbuffer	*_accel_ringbuffer;
	struct accel_scale  _accel_scale;
	f32			_accel_range_scale;
	f32			_accel_range_m_s2;

	ringbuffer	*_gyro_ringbuffer;
	struct gyro_scale   _gyro_scale;
	f32			_gyro_range_scale;
	f32			_gyro_range_rad_s;

	u32		_dlpf_freq;
	u32		_sample_rate;
	// last temperature reading for print_info()
	f32	    _last_temperature;
	u8	    _product;	/** product code */

	enum rotation	_rotation;

	math::lowpassfilter2p	_accel_filter_x;
	math::lowpassfilter2p	_accel_filter_y;
	math::lowpassfilter2p	_accel_filter_z;
	math::lowpassfilter2p	_gyro_filter_x;
	math::lowpassfilter2p	_gyro_filter_y;
	math::lowpassfilter2p	_gyro_filter_z;


	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define MPU6000_NUM_CHECKED_REGISTERS 9
	static const u8	_checked_registers[MPU6000_NUM_CHECKED_REGISTERS];
	u8			_checked_values[MPU6000_NUM_CHECKED_REGISTERS];
	u8			_checked_next;

public:
    s32 probe(spi *pspi, gpio *gpio_cs);
    s32 remove(void);

public:
    s32 read_accel(u8 *buf, u32 size);
    s32 read_gyro(u8 *buf, u32 size);

public:
	virtual void run(void *parg);

public:
    s32 init(void);
    s32 reset(void);
    void measure(void);
    s16 s16_from_bytes(u8 bytes[]);
    s32 calibrate_gyro(void);


    s32 calibrate_accel(void);
    s32 calibratie_accel_measurements(float (&accel_offs)[3], float (&accel_T)[3][3], u32 *active_sensors);
    s32 detect_orientation(void);
    s32 read_accelerometer_avg(float (&accel_avg)[6][3], u32 orient, u32 samples_num);
    s32 mat_invert3(float src[3][3], float dst[3][3]);
    s32 calculate_calibration_values(float (&accel_ref)[6][3], float (&accel_T)[3][3], float (&accel_offs)[3], float g);


public:
	s32 self_test(void);
	s32 chip_self_test(void);

public:
	s32 set_gyro_fsr(u16 fsr);
	s32 set_accel_fsr(u8 fsr);
	void set_dlpf_filter(u16 frequency_hz);
	void set_sample_rate(u32 desired_sample_rate_hz);
    s32 set_accel_range(u32 max_g_in);

	s32 get_gyro_raw(s16 *gyro);
	s32 get_accel_raw(s16 *accel);
	s32 get_temperature(f32 *temperature);

	u16 orientation_matrix_to_scalar(s8 *mtx);
	u16 row_2_scale(s8 *row);
	s8 	dmp_get_data(f32 *pitch, f32 *roll, f32 *yaw);

private:
	void write_checked_reg(u32 reg, u8 value);
	inline s32 read_reg8(u8 reg);
	inline s32 write_reg8(u8 reg, u8 data);
	s32 read_reg(u8 reg, u8 *buf, u8 len);
	s32 write_reg(u8 reg, u8 *buf, u8 len);
};


}


/***********************************************************************
** End of file
***********************************************************************/

