/*******************************Copyright (c)***************************
**
** Porject name:	robot
** Created by:		
** Created date:	
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "robot_type.h"
#include "robot_misc.h"

//#include "drivers/device.h"
//#include "drivers/core.h"
//#include "drivers/flash.h"
//#include "drivers/uart/uart.h"

//using namespace driver;

namespace app {

class rbt_system
{
public:
	rbt_system(void);
	~rbt_system(void);

public:
    /**
     *  @enum  system_mode
     *  @brief 平台模式
     */
    enum rbt_system_mode
    {
        M_AUTO = 0,
        M_BOOTLOADER,
    };

public:
    enum rbt_system_mode _mode;
    //uart          	*_puart1;

public:
    enum rbt_system_mode    get_mode(void)		{ return _mode; }
    //uart        *get_uart1(void)		    { return _puart1; }


protected:
    static rbt_system *s_pactive_instance;

public:
    s32 init(enum rbt_system_mode mode);
    virtual s32 init(void);
    virtual void start(void);
    virtual s32 exit(void);

    // 获取唯一全局实例
    inline static rbt_system* get_instance(void) { return s_pactive_instance; }
};

}

/***********************************************************************
** End of file
***********************************************************************/


