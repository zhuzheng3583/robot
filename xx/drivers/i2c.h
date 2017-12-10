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

#include "core.h"
#include "device.h"
#include "dma.h"

namespace driver {

#define I2C_M_TEN               0x0010	/* this is a ten bit chip address */
#define I2C_M_RD                0x0001	/* read data, from slave to master */
#define I2C_M_STOP              0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART           0x4000	/* if I2C_FUNC_NOSTART */
#define I2C_M_REV_DIR_ADDR      0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK        0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK         0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN          0x0400  /* length will be first received byte */

struct i2c_msg {
	u16 addr;	/* slave address */
	u16 flags;  
	u16 len;		/* msg length	 */
	u8 *buf;		/* pointer to msg data */
};

 
class i2c : public device
{
public:
	i2c(PCSTR name, s32 id);
	~i2c(void);

public:
    dma *_dmatx;
    dma *_dmarx;
    s32 _dma_tx_id;
    s32 _dma_rx_id;

    s32 _flag_tx;
    s32 _flag_rx;
    
public:
    s32 probe(void);
    s32 remove(void);
    
public:
    s32 transfer(struct i2c_msg *msgs, int num);
    
    s32 reset(void);
    s32 self_test(void);
    
public:
    void write_reg(u16 addr, u8 reg, u8 *buf, u32 len);
    void read_reg(u16 addr, u8 reg, u8 *buf, u32 len);    
};

}
/***********************************************************************
** End of file
***********************************************************************/

