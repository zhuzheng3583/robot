/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/11
** Modified by:
** Modified date:
** Descriptions:
**				logic_dev_id
DMA1
DMA1_Stream0	[0,7]
DMA1_Stream1	[8,15]
DMA1_Stream2	[16,23]
DMA1_Stream3	[24,31]
DMA1_Stream4	[32,39]
DMA1_Stream5	[40,47]
DMA1_Stream6	[48,55]
DMA1_Stream7	[56,63]
DMA2
DMA2_Stream0	[64,71]
DMA2_Stream1	[72,79]
DMA2_Stream2	[80,87]
DMA2_Stream3	[88,95]
DMA2_Stream4	[96,103]
DMA2_Stream5	[104,111]
DMA2_Stream6	[112,119]
DMA2_Stream7	[120,127]

DMA_CHANNEL_0
DMA_CHANNEL_1
DMA_CHANNEL_2
DMA_CHANNEL_3
DMA_CHANNEL_4
DMA_CHANNEL_5
DMA_CHANNEL_6
DMA_CHANNEL_7
***********************************************************************/
#include "dma.h"

//id = [0,127]
#define DMA_CNT          		(2)
#define STREAM_CNT_PER_DMA		(8)
#define CHANNEL_CNT_PER_STREAM	(8)
#define CHANNEL_CNT_PER_DMA	(STREAM_CNT_PER_DMA * CHANNEL_CNT_PER_STREAM)
#define MAX_LINE_CHANNEL_ID	(CHANNEL_CNT_PER_DMA * DMA_CNT - 1)
#define MIN_LINE_CHANNEL_ID	(0)

namespace driver {

struct stm32_dma_hw_table
{
	DMA_TypeDef 			*DMAx;
	IRQn_Type 			IRQn;
	DMA_HandleTypeDef		DMA_Handle;
};

static struct stm32_dma_hw_table dma_hw_table[] = {
	[0] = { NULL },
	[12] = {
		.DMAx = DMA1,
		.IRQn = DMA1_Stream1_IRQn,
		.DMA_Handle = {
			.Instance = DMA1_Stream1,
			.Init = {
				.Channel		= DMA_CHANNEL_4,
				.Mode 			= DMA_NORMAL,
				.FIFOMode       = DMA_FIFOMODE_DISABLE,
				.FIFOThreshold	= DMA_FIFO_THRESHOLD_FULL,
				.MemBurst      	= DMA_MBURST_INC4,
				.PeriphBurst    = DMA_PBURST_INC4,
			},
		},
	},
	[28] = {
		.DMAx = DMA1,
		.IRQn = DMA1_Stream3_IRQn,
		.DMA_Handle = {
			.Instance = DMA1_Stream3,
			.Init = {
				.Channel		= DMA_CHANNEL_4,
				.Mode 			= DMA_NORMAL,
				.FIFOMode       = DMA_FIFOMODE_DISABLE,
				.FIFOThreshold	= DMA_FIFO_THRESHOLD_FULL,
				.MemBurst      	= DMA_MBURST_INC4,
				.PeriphBurst    = DMA_PBURST_INC4,
			},
		},
	},
	[44] = {
		.DMAx = DMA1,
		.IRQn = DMA1_Stream5_IRQn,
		.DMA_Handle = {
			.Instance = DMA1_Stream5,
			.Init = {
				.Channel		= DMA_CHANNEL_4,
				.Mode 			= DMA_NORMAL,
				.FIFOMode       = DMA_FIFOMODE_DISABLE,
				.FIFOThreshold	= DMA_FIFO_THRESHOLD_FULL,
				.MemBurst      	= DMA_MBURST_INC4,
				.PeriphBurst    = DMA_PBURST_INC4,
			},
		},
	},
	[52] = {
		.DMAx = DMA1,
		.IRQn = DMA1_Stream6_IRQn,
		.DMA_Handle = {
			.Instance = DMA1_Stream6,
			.Init = {
				.Channel		= DMA_CHANNEL_4,
				.Mode 			= DMA_NORMAL,
				.FIFOMode       = DMA_FIFOMODE_DISABLE,
				.FIFOThreshold	= DMA_FIFO_THRESHOLD_FULL,
				.MemBurst      	= DMA_MBURST_INC4,
				.PeriphBurst    = DMA_PBURST_INC4,
			},
		},
	},
    [77] = {
		.DMAx = DMA2,
		.IRQn = DMA2_Stream1_IRQn,
		.DMA_Handle = {
			.Instance = DMA2_Stream1,
			.Init = {
				.Channel		= DMA_CHANNEL_5,
				.Mode 			= DMA_NORMAL,
				.FIFOMode       = DMA_FIFOMODE_DISABLE,
				.FIFOThreshold	= DMA_FIFO_THRESHOLD_FULL,
				.MemBurst      	= DMA_MBURST_INC4,
				.PeriphBurst    = DMA_PBURST_INC4,
			},
		},
	},
    
    [83] = {
        .DMAx = DMA2,
        .IRQn = DMA2_Stream2_IRQn,
        .DMA_Handle = {
        .Instance = DMA2_Stream2,
            .Init = {
                .Channel        = DMA_CHANNEL_3,
                .Mode           = DMA_NORMAL,
                .FIFOMode       = DMA_FIFOMODE_DISABLE,
                .FIFOThreshold  = DMA_FIFO_THRESHOLD_FULL,
                .MemBurst       = DMA_MBURST_INC4,
                .PeriphBurst    = DMA_PBURST_INC4,
            },
        },
    },
    [91] = {
        .DMAx = DMA2,
        .IRQn = DMA2_Stream3_IRQn,
        .DMA_Handle = {
        .Instance = DMA2_Stream3,
            .Init = {
                .Channel        = DMA_CHANNEL_3,
                .Mode           = DMA_NORMAL,
                .FIFOMode       = DMA_FIFOMODE_DISABLE,
                .FIFOThreshold  = DMA_FIFO_THRESHOLD_FULL,
                .MemBurst       = DMA_MBURST_INC4,
                .PeriphBurst    = DMA_PBURST_INC4,
            },
        },
    },
    [108] = {
        .DMAx = DMA2,
        .IRQn = DMA2_Stream5_IRQn,
        .DMA_Handle = {
        .Instance = DMA2_Stream5,
            .Init = {
                .Channel        = DMA_CHANNEL_4,
                .Mode           = DMA_NORMAL,
                .FIFOMode       = DMA_FIFOMODE_DISABLE,
                .FIFOThreshold  = DMA_FIFO_THRESHOLD_FULL,
                .MemBurst       = DMA_MBURST_INC4,
                .PeriphBurst    = DMA_PBURST_INC4,
            },
        },
    },
    [124] = {
        .DMAx = DMA2,
        .IRQn = DMA2_Stream7_IRQn,
        .DMA_Handle = {
        .Instance = DMA2_Stream7,
            .Init = {
                .Channel        = DMA_CHANNEL_4,
                .Mode           = DMA_NORMAL,
                .FIFOMode       = DMA_FIFOMODE_DISABLE,
                .FIFOThreshold  = DMA_FIFO_THRESHOLD_FULL,
                .MemBurst       = DMA_MBURST_INC4,
                .PeriphBurst    = DMA_PBURST_INC4,
            },
        },
    },
};

dma::dma(PCSTR name, s32 id) :
	device(name, id)
{
	_dma_id = _id / CHANNEL_CNT_PER_DMA + 1;
	_stream_id = _id / STREAM_CNT_PER_DMA - (_dma_id-1)*STREAM_CNT_PER_DMA;
	_channel_id = (_id - (_dma_id-1)*STREAM_CNT_PER_DMA*CHANNEL_CNT_PER_STREAM) \
        / CHANNEL_CNT_PER_STREAM;
}

dma::~dma(void)
{
}

s32 dma::probe(void)
{
	//ASSERT(_id <= DMA_CHANNEL_MAX_NUM && _id > 0);
	DMA_HandleTypeDef *hdma = &dma_hw_table[_id].DMA_Handle;
	if(HAL_DMA_GetState(hdma) != HAL_DMA_STATE_RESET)
	{
		//ERR("%s: failed HAL_DMA_GetState.\n", _name);
		goto fail0;
	}

	switch(_dma_id)
    	{
	case 1:
		__HAL_RCC_DMA1_CLK_ENABLE();
		break;
	case 2:
		__HAL_RCC_DMA2_CLK_ENABLE();
		break;
	default:
		break;
    	}

	_irq = dma_hw_table[_id].IRQn;
	_handle = (u32)hdma;

	/*  Configure the NVIC for DMA */
  	/* NVIC configuration for DMA transfer complete interrupt*/
	device::request_irq(_irq, this);
	device::enable_irq(_irq);
	//INF("%s: probe success.\n", _name);
	return 0;

fail0:
	return -1;
}

s32 dma::remove(void)
{
	DMA_HandleTypeDef *hdma = (DMA_HandleTypeDef *)_handle;

	device::disable_irq(_irq);

	if(HAL_DMA_DeInit(hdma)!= HAL_OK) {
        goto fail0;
	}
	_handle = NULL;

	return 0;

fail0:
    return -1;
}


s32 dma::config(enum dma_dir mode, enum dma_align align, enum dma_pri priority,
	s32 en_src_step, s32 en_dst_step)
{
	DMA_HandleTypeDef *hdma = (DMA_HandleTypeDef *)_handle;
	/* Configure the DMA */
	/* Configure the DMA handler for Transmission process */
    switch (mode)
    {
	case DMA_DIR_PERIPH_TO_MEM:
		hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
        	hdma->Init.MemInc = en_src_step ? DMA_PINC_ENABLE : DMA_PINC_DISABLE;
        	hdma->Init.PeriphInc = en_dst_step ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
        	break;
    	case DMA_DIR_MEM_TO_PERIPH:
		hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
        	hdma->Init.MemInc = en_src_step ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
        	hdma->Init.PeriphInc = en_dst_step ? DMA_PINC_ENABLE : DMA_PINC_DISABLE;
		break;
    	case DMA_DIR_MEM_TO_MEM:
		hdma->Init.Direction = DMA_MEMORY_TO_MEMORY;
		hdma->Init.MemInc = en_src_step ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
		hdma->Init.PeriphInc = en_dst_step ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
        break;
    	default:
        //ERR("%s: error DMA direct mode.\n", _name);
        CAPTURE_ERR();
        break;
    }

	switch (align)
	{
    	case DMA_ALIGN_BYTE:
  		hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  		hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		break;
    	case DMA_ALIGN_HALF_WORD:
  		hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  		hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        	break;
    	case DMA_ALIGN_WORD:
  		hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  		hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        	break;
    	default:
		//ERR("%s: error DMA align.\n", _name);
		CAPTURE_ERR();
		break;
	}

 	switch (priority)
	{
    	case DMA_PRI_LOW:
		hdma->Init.Priority = DMA_PRIORITY_LOW;
		break;
    	case DMA_PRI_MEDIUM:
  		hdma->Init.Priority = DMA_PRIORITY_MEDIUM;
        	break;
    	case DMA_PRI_HIGH:
  		hdma->Init.Priority = DMA_PRIORITY_HIGH;
        	break;
    	case DMA_PRI_VERY_HIGH:
  		hdma->Init.Priority = DMA_PRIORITY_VERY_HIGH;
        	break;
    	default:
		//ERR("%s: error DMA priority.\n", _name);
		CAPTURE_ERR();
		break;
	}

  	if(HAL_DMA_Init(hdma) != HAL_OK) {
		//ERR("%s: faile to config.\n", _name);
		CAPTURE_ERR();
		return -1;
  	}

	return 0;
}

//stm32 cube库在每个外设中已经封装了DMA相关，暂不使用改方法
s32 dma::async_copy(void *psrc_addr, void *pdst_addr, u32 size)
{

	DMA_HandleTypeDef *hdma = (DMA_HandleTypeDef *)_handle;
#if 0
	DBG("dma transfer size = %d.\n", size);
	/* Set the UART DMA transfer complete callback */
    	hdma->XferCpltCallback = UART_DMATransmitCplt;

    	/* Set the UART DMA Half transfer complete callback */
    	hdma->XferHalfCpltCallback = UART_DMATxHalfCplt;

    	/* Set the DMA error callback */
    	hdma->XferErrorCallback = UART_DMAError;
#endif
    	/* Enable the DMA channel */
    	HAL_DMA_Start_IT(hdma, (uint32_t)psrc_addr, (uint32_t)pdst_addr, size);

	return 0;
}

//stm32 cube库在每个外设中已经封装了DMA相关，暂不使用改方法
s32 dma::wait_complete(s32 timeoutms)
{
#if 1 //polling
	HAL_StatusTypeDef status;
	status = HAL_DMA_PollForTransfer((DMA_HandleTypeDef *)_handle, HAL_DMA_FULL_TRANSFER, (uint32_t)timeoutms);
	if (status != HAL_OK) {
		return -1;
	}
#else
	//IT
#endif
	return 0;
}

s32 dma::get_leftover_count(void)
{
    DMA_HandleTypeDef *p = (DMA_HandleTypeDef *)_handle;
    return (s32)(p->Instance->NDTR);
}


void dma::isr(void)
{
	HAL_DMA_IRQHandler((DMA_HandleTypeDef *)_handle);
}

//TODO 读取传输剩余字节数

}
/***********************************************************************
** End of file
***********************************************************************/

