/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "drivers/device.h"
#include "drivers/core.h"
#include "drivers/flash.h"
#include "drivers/uart/uart.h"
#include "drivers/uart/uart_polling.h"
#include "drivers/uart/uart_int.h"
#include "drivers/uart/uart_dma.h"
#include "drivers/i2c.h"
#include "drivers/usb_dev.h"
#include "drivers/sensorhub.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm.h"
#include "drivers/spi.h"

#include "drivers/accel/mpu6000.h"
#include "drivers/baro/ms5611.h"
#include "drivers/magn/hmc5883.h"
#include "drivers/gps/gps.h"
#include "drivers/gps/ashtech.h"
#include "drivers/motor.h"
#include "drivers/rc/sbus.h"


#include "demo_main.h"


#include "logger.h"
#include "heartbeat.h"
#include "terminal.h"
#include "autopilot.h"
#include "calibration.h"

#include "os/kernel.h"
#include "os/msgque.h"
#include "os/semaphore.h"

#include "niming.h"

using namespace driver;
using namespace os;

namespace app {

class leader_system
{
public:
	leader_system(void);
	~leader_system(void);

public:
    /**
     *  @enum  leader_system_mode
     *  @brief 平台模式
     */
    enum leader_system_mode
    {
        M_AUTO = 0,
        M_BOOTLOADER,
        M_AIRCRAFT,
    };

public:
    enum leader_system_mode _mode;
    uart                    *_puart1;
    uart                    *_puart2;
    uart                    *_puart3;
    uart                    *_puart6;

    i2c                     *_i2c1;

    spi                     *_spi1;
    gpio                    *_mpu6000_gpio_cs;
    gpio                    *_ms5611_gpio_cs;
    mpu6000		            *_mpu6000;
    ms5611					*_ms5611;
	i2c                     *_i2c2;
	hmc5883					*_hmc5883;

    pwm                     *_pwm1;
    pwm                     *_pwm4;
    motor                   *_motor1;
    motor                   *_motor2;
    motor                   *_motor3;
    motor                   *_motor4;
    motor                   *_motor5;
    motor                   *_motor6;
    motor                   *_motor7;
    motor                   *_motor8;

    gpio                    *_led_blue;
	gpio                    *_led_amber;
    gpio                    *_gpio_irq;

    ashtech                 *_pashtech;
    gps                     *_pgps;
    
    sbus                    *_sbus;

    flash                   *_pflash;

    usb_dev                 *_usb_dev;
    sensorhub               *_sensorhub;

    timer                   *_timer;

    logger                  *_logger;
    heartbeat               *_heartbeat;
    terminal				*_terminal;
    autopilot			    *_autopilot;
    calibration             *_calibration;

    niming                  *_niming;

public:
    enum leader_system_mode    get_mode(void)		{ return _mode; }
    uart        *get_uart1(void)		    { return _puart1; }
    uart        *get_uart2(void)		    { return _puart2; }
    uart        *get_uart3(void)		    { return _puart3; }

    flash       *get_flash(void)            { return _pflash; }

    i2c         *get_i2c1(void)             { return _i2c1; }
    i2c         *get_i2c2(void)             { return _i2c2; }

    usb_dev     *get_usb_dev(void)          { return _usb_dev; }
    sensorhub   *get_sensorhub(void)        { return _sensorhub; }

    mpu6000   	*get_mpu6000(void)        	{ return _mpu6000; }
	ms5611		*get_ms5611(void)        	{ return _ms5611; }
	hmc5883		*get_hmc5883(void)        	{ return _hmc5883; }

    ashtech		*get_ashtech(void)        	{ return _pashtech; }
    
    sbus        *get_sbus(void)        	     { return _sbus; }
    
	gpio 		*get_led_blue(void)			{ return _led_blue; }
	gpio		*get_led_amber(void) 		{ return _led_amber; }

    logger		*get_logger(void)        	{ return _logger; }

    niming      *get_niming(void)           { return _niming; }
protected:
    static leader_system *s_pactive_instance;

public:
    s32 init(enum leader_system_mode mode);
    virtual s32 init(void);
    virtual void start(void);
    virtual s32 exit(void);

    // 获取唯一全局实例
    inline static leader_system* get_instance(void) { return s_pactive_instance; }
};

}

/***********************************************************************
** End of file
***********************************************************************/


