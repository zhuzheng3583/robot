/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/11
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

enum dma_dir {
    DMA_DIR_PERIPH_TO_MEM	= 0,
    DMA_DIR_MEM_TO_PERIPH 	= 1,
    DMA_DIR_MEM_TO_MEM 	= 2,
};

enum dma_align {
    DMA_ALIGN_BYTE      = 0,
    DMA_ALIGN_HALF_WORD = 1,
    DMA_ALIGN_WORD      = 2,
};

enum dma_pri {
    DMA_PRI_LOW        = 0,
    DMA_PRI_MEDIUM     = 1,
    DMA_PRI_HIGH       = 2,
    DMA_PRI_VERY_HIGH  = 3,
};


class dma : public device
{
public:
	dma(PCSTR name, s32 id);
	~dma(void);

public:
    s32 _dma_id;
    s32 _stream_id;
    s32 _channel_id;

public:
    s32 probe(void);
    s32 remove(void);

    s32 config(enum dma_dir mode, enum dma_align align, enum dma_pri priority,
        s32 en_src_step, s32 en_dst_step);
    s32 async_copy(void *psrc_addr, void *pdst_addr, u32 size);
    s32 wait_complete(s32 timeoutms);

	s32 get_leftover_count(void);
public:
    virtual void isr(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/







