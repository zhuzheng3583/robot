/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/14
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

#define  PERIOD_VALUE       (u32)(665 - 1)  			/* Period Value 100% */
#define  PULSE1_VALUE       (u32)(PERIOD_VALUE*50/100)   	/* Capture Compare 1 Value 50% */
#define  PULSE2_VALUE       (u32)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value 37.5% */
#define  PULSE3_VALUE       (u32)(PERIOD_VALUE*25/100) 	/* Capture Compare 3 Value 25% */
#define  PULSE4_VALUE       (u32)(PERIOD_VALUE*12.5/100) /* Capture Compare 4 Value 12.5% */


#define PWM_CHANNEL_1       0
#define PWM_CHANNEL_2       1
#define PWM_CHANNEL_3       2
#define PWM_CHANNEL_4       3

namespace driver {

class pwm : public device
{
public:
	pwm(PCSTR devname, s32 devid);
	~pwm(void);

public:
	u32 _channel;
	u32 _dutycycle;

public:
    s32 probe(void);
    s32 remove(void);

public:
    s32 set_dutycycle(u32 channel, u32 dutycycle);
    s32 start(u32 channel);

public:
	s32 self_test(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/

