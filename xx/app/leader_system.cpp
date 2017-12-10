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
#include "leader_system.h"

#include "bootloader.h"
#include "aircraft.h"

#include "demo_main.h"

#include "cmsis_os.h"


namespace app {

leader_system *leader_system::s_pactive_instance = NULL;

leader_system::leader_system(void)
{

}

leader_system::~leader_system(void)
{
	delete s_pactive_instance;
	s_pactive_instance = NULL;
}

s32 leader_system::init(enum leader_system_mode mode)
{
	// TODO: 调试时在此强制指定系统模式
	//mode = M_BOOTLOADER;

	_mode = mode;
	switch(mode)
	{
	case M_AUTO:
#if defined(BOOTLOADER)
        s_pactive_instance = new bootloader;
		_mode = M_BOOTLOADER;
#elif defined(AIRCRAFT)
		s_pactive_instance = new aircraft;
		_mode = M_AIRCRAFT;
#else
#error Pre-defined macros required.
#endif
        break;
	case M_BOOTLOADER:
		s_pactive_instance = new bootloader;
		break;
	case M_AIRCRAFT:
		s_pactive_instance = new aircraft;
		break;
	default:
		break;
	}

	if (s_pactive_instance) {
		return s_pactive_instance->init();
	}

	return -1;
}

void timer_func(void *arg)
{
	INF("timer_func: success call!\n");
}


s32 leader_system::init(void)
{
#if USE_STM32F4_DEMO
	demo_main();
#endif

	s32 ret = 0;
	ret = core::init();
	ret = device::irq_init();
	kernel::systick_config();

    _logger = new logger;
	//_logger->self_test();
	//在此之前不能使用log输出
    
    //core::self_test();
#if 0

	_pflash = new flash("flash", -1);
	_pflash->probe();

	_i2c1 = new i2c("i2c-1", 1);
	_i2c1->probe();
	_i2c1->self_test();

	_usb_dev = new usb_dev("usb_dev", -1);
	_usb_dev->probe();
	//_usbd->self_test();

	_sensorhub = new sensorhub("sensorhub", -1);
	_sensorhub->probe(_usb_dev);
	//_sensorhub->self_test();

	_gpio_irq = new gpio("gpio-0", 0);
	_gpio_irq->probe();
	_gpio_irq->set_gpio_to_irq();
	//while(1);

	_timer = new timer("timer-2", 2);
	_timer->probe();
	//_timer->self_test();
	_timer->set_timeout(1000);
	_timer->set_function(timer_func, (void *)1111);
//	_timer->start();
    while(1);
#endif

	if(ret < 0) {
		INF("failed to leader_system::init");
		CAPTURE_ERR();
	}

	return 0;
}

void leader_system::start(void)
{
	//kernel::start();
}

s32 leader_system::exit(void)
{
	//kernel::exit();
	return 0;
}

}


/***********************************************************************
** End of file
***********************************************************************/

