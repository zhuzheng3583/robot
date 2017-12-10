/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/12
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

#define GPIO_PIN_NUM_PER_BANK       16

//id = [0, 95]
#define GPIO_PIN_NUM                96

#define VHIGH						(1)
#define VLOW						(0)
namespace driver {

class gpio : public device
{
public:
	gpio(PCSTR name, s32 id);
	~gpio(void);

public:
	s32 _bank;		// bank±‡∫≈(0,1,2,3,4,5==>A,B,C,D,E,F)
	s32 _index;		// index±‡∫≈(0~15)

public:
    s32 probe(void);
    s32 remove(void);

public:
    s32 set_direction_input(void);
    s32 set_direction_output(void);
    s32 set_gpio_to_irq(void);

    void set_value(u8 value);
    u8 get_value(void);

public:
    s32 self_test(void);

public:
    virtual void isr(void);

/* TODO: GPIOÕ‚≤ø÷–∂œ */
};

}
/***********************************************************************
** End of file
***********************************************************************/


