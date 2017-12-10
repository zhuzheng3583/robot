/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/04/08
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "aircraft.h"

namespace app {

aircraft::aircraft(void) :
	_into_calibrate(false)
{
	_params.name = "aircraft";
	_params.priority = 0;
	_params.stacksize = 256;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

aircraft::~aircraft(void)
{

}

s32 aircraft::init(void)
{
	leader_system::init();
	INF("========Init LeaderUAV Aircraft App ========\n");

	kernel::init();
    
    aircraft::create(NULL);
    
	return 0;
}

void aircraft::start(void)
{
	INF("========Start LeaderUAV Aircraft App ========\n");
    
    kernel::start();
}

s32 aircraft::exit(void)
{
	return -1;
}

void aircraft::run(void *parg)
{    
  	_puart2 = new uart("uart-2", 2);
	_puart2->probe();
	//_puart2->self_test();
    
    _logger->attach(_puart2);
    _logger->create(NULL);
    
    /*
	 * 初始化LED
	 * note P2 Pro v1.1: PE12(76):LED_BLUE PE15(79):LED_AMBER
	 */
    _led_blue = new gpio("led_blue", 76);
    _led_amber = new gpio("led_amber", 79);
    _led_blue->probe();
    _led_amber->probe();
    _led_blue->set_direction_output();
    _led_amber->set_direction_output();
    _led_blue->set_value(VLOW);
    _led_amber->set_value(VLOW);

	/*
	 * 初始化RC
	 */
    _puart6 = new uart("uart-6", 6);
	_puart6->probe();
    //_puart6->self_test();
    _sbus = new sbus("sbus", -1);
    _sbus->probe(_puart6);

	/*
	 * 初始化匿名通讯协议
	 */
    _puart3 = new uart("uart-3", 3);
	_puart3->probe();
	//_puart3->self_test();
	_niming = new niming;
	_niming->attach(_puart3);

	/*
	 * 初始化GPS ashtech协议
	 */
	_puart1 = new uart("uart-1", 1);
	_puart1->probe();
	//_puart1->self_test();
	//_pgps = new gps("gps", -1);
	//_pgps->probe(_puart1);
	_pashtech = new ashtech("ashtech", -1);
	_pashtech->probe(_puart1);

	/*
	 * 初始化SPI1,用于访问mpu6000和ms5611(accel, gyro, baro)
	 */
	_spi1 = new spi("spi-1", 1);
	_spi1->probe();

	/*
	 * 初始化mpu6000(accel, gyro)
	 */
	_mpu6000_gpio_cs = new gpio("mpu6000_gpio_cs-34", 34);
	_mpu6000_gpio_cs->probe();
	_mpu6000 = new mpu6000("mpu6000", -1);
	_mpu6000->probe(_spi1, _mpu6000_gpio_cs);

	/*
	 * 初始化ms5611(baro)
	 */
	_ms5611_gpio_cs = new gpio("ms5611_gpio_cs-55", 55);
	_ms5611_gpio_cs->probe();
    _ms5611 = new ms5611("ms5611", -1);
    _ms5611->probe(_spi1, _ms5611_gpio_cs);

	/*
	 * 初始化hmc5883(mag)
	 */
	_i2c2 = new i2c("i2c-2", 2);
    _i2c2->probe();
    _hmc5883 = new hmc5883("hmc5883", -1);
    _hmc5883->probe(_i2c2, HMC5883_SLAVE_ADDRESS);

	/*
	 * 初始化电机[1,8]
	 */
	_pwm1 = new pwm("timer-1", 1);
	_pwm1->probe();
	_motor1 = new motor("motor-1", 1);
	_motor1->probe(_pwm1, PWM_CHANNEL_4);
	_motor2 = new motor("motor-2", 2);
	_motor2->probe(_pwm1, PWM_CHANNEL_3);
	_motor3 = new motor("motor-3", 3);
	_motor3->probe(_pwm1, PWM_CHANNEL_2);
	_motor4 = new motor("motor-4", 4);
	_motor4->probe(_pwm1, PWM_CHANNEL_1);

	_pwm4 = new pwm("timer-4", 4);
	_pwm4->probe();
	_motor5 = new motor("motor-5", 5);
	_motor5->probe(_pwm4, PWM_CHANNEL_4);
	_motor6 = new motor("motor-6", 6);
	_motor6->probe(_pwm4, PWM_CHANNEL_3);
	_motor7 = new motor("motor-7", 7);
	_motor7->probe(_pwm4, PWM_CHANNEL_2);
	_motor8 = new motor("motor-8", 8);
	_motor8->probe(_pwm4, PWM_CHANNEL_1);

	_motor1->set_lock(false);
	_motor1->set_throttle(50);
	_motor2->set_lock(false);
	_motor2->set_throttle(50);
	_motor3->set_lock(false);
	_motor3->set_throttle(50);
	_motor4->set_lock(false);
	_motor4->set_throttle(50);
	_motor5->set_lock(false);
	_motor5->set_throttle(50);
	_motor6->set_lock(false);
	_motor6->set_throttle(50);
	_motor7->set_lock(false);
	_motor7->set_throttle(50);
	_motor8->set_lock(false);
	_motor8->set_throttle(50);  
    
    /*
	 * 创建线程
	 */
	_heartbeat = new heartbeat;
	_terminal = new terminal;
	_autopilot = new autopilot;
	_calibration = new calibration;

	_pashtech->create(NULL);
    _sbus->create(NULL);
    _mpu6000->create(NULL);
    _hmc5883->create(NULL);
    _ms5611->create(NULL);
	_heartbeat->create(NULL);
	_terminal->create(NULL);

    _into_calibrate = false;
	if (_into_calibrate == true) {
		_calibration->create(NULL);
	} else {
		_autopilot->create(NULL);
	}  

    //msleep(999999);
    
    aircraft::t_delete();
}

}


/***********************************************************************
** End of file
***********************************************************************/

