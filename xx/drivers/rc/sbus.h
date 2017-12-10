/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/08/22
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "device.h"
#include "uart.h"
#include "rc_input.h"
#include "ringbuffer.h"

#include "os/thread.h"

#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2
#define SBUS1_FRAME_DELAY	14000
/*
  Measured values with Futaba FX-30/R6108SB:
    -+100% on TX:  PCM 1.100/1.520/1.950ms -> SBus raw values: 350/1024/1700  (100% ATV)
    -+140% on TX:  PCM 0.930/1.520/2.112ms -> SBus raw values:  78/1024/1964  (140% ATV)
    -+152% on TX:  PCM 0.884/1.520/2.160ms -> SBus raw values:   1/1024/2047  (140% ATV plus dirty tricks)
*/

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

namespace driver {

class sbus : public device, public thread
{
public:
    sbus(PCSTR name, s32 id);
    ~sbus(void);

public:
    uart *_puart;
    ringbuffer *_sbus_ringbuffer;
    struct rc_input_values  _sbus_rc_input_values;
    
    u32 _frame_drops;
    u8 _frame[SBUS_FRAME_SIZE];
    
    bool _failsafe;
    bool _frame_drop;
public:
    s32 probe(uart *puart);
    s32 remove(void);

public:
    s32 init(void);
    s32 reset(void);
    void measure(void);
    
public:
    virtual s32 read(u8 *buf, u32 size);    
    
public:
	virtual void run(void *parg);
};

}
/***********************************************************************
** End of file
***********************************************************************/
