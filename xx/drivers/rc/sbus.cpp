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
#include "sbus.h"
#include "core.h"

//一、协议说明：
//串口配置为波特率100kbps，8位数据，偶校验(even)，2位停止位，无流控。
//链接https://mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/说明了S-bus帧格式。
//每帧25个字节，按照如下顺序排列：
//[startbyte] [data1] [data2] .... [data22] [flags][endbyte]
//起始字节startbyte = 11110000b (0xF0)，但实际上用STM32（据说ARM核）收到的是0x0F。
//中间22个字节就是16个通道的数据了，为什么是16个通道？因为22x8=11x16，每个通道用11bit表示，范围是0-2047。不信看波形图：

#define BAUD9600_DELAY_US       ((1*1000*1000) / 9600)
#define BAUD100000_DELAY_US     ((1*1000*1000) / 100000)
#define BAUD115200_DELAY_US     ((1*1000*1000) / 115200)

#define DEFAULT_BAUD_DELAY_US   (BAUD9600_DELAY_US)     

namespace driver {
  
sbus::sbus(PCSTR devname, s32 devid) :
	device(devname, devid)
{
	_params.name = "sbus_thread";
	_params.priority = 0;
	_params.stacksize = 256;
	_params.func = (void *)thread::func;
	_params.parg = (thread *)this;
}

sbus::~sbus(void)
{

}

s32 sbus::probe(uart *puart)
{
    _puart = puart;
    _puart->open();
#if 0
    u8 c[25] = {0};
    while (1) {
        _puart->read(c, sizeof(c));
        for (u32 i = 0; i < sizeof(c); i++) {
            DBG("c[%d] = 0x%02x ", i, c[i]);
        }
    }
#endif
    sbus::init();
    
	return 0;
}


s32 sbus::remove(void)
{
	return 0;
}


s32 sbus::init(void)
{
	/* allocate basic report buffers */
	_sbus_ringbuffer = new ringbuffer(2, sizeof(struct rc_input_values));
	if (_sbus_ringbuffer == NULL) {
		goto out;
	}

	if (reset() != 0) {
		ERR("%s: failed to reset.\n", _devname);
		goto out;
	}

	/* discard any stale data in the buffers */
	_sbus_ringbuffer->flush();

	return 0;

out:
	return -1;
}

s32 sbus::reset(void)
{

}

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

void sbus::measure(void)
{

    struct rc_input_values rc_report;
    u32 cnt = 0;
    u16 values[sizeof(_frame)];
    cnt = _puart->read(_frame, sizeof(_frame));
    if (cnt != sizeof(_frame)) {
        return;
    }
  
    /* check frame boundary markers to avoid out-of-sync cases */
	if ((_frame[0] != 0x0f)) {
		_frame_drops++;
		return ;
	}

	switch (_frame[24]) {
	case 0x00:
		/* this is S.BUS 1 */
		break;
	case 0x03:
		/* S.BUS 2 SLOT0: RX battery and external voltage */
		break;
	case 0x83:
		/* S.BUS 2 SLOT1 */
		break;
	case 0x43:
	case 0xC3:
	case 0x23:
	case 0xA3:
	case 0x63:
	case 0xE3:
		break;
	default:
		/* we expect one of the bits above, but there are some we don't know yet */
		break;
	}

	/* we have received something we think is a frame */
	//last_frame_time = frame_time;

	unsigned chancount = SBUS_INPUT_CHANNELS;

	/* use the decoder matrix to extract channel data */
	for (unsigned channel = 0; channel < chancount; channel++) {
		unsigned value = 0;

		for (unsigned pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = _frame[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}

		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		values[channel] = (u16)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
        rc_report.values[channel] = values[channel];
	}

#if 0
	/* decode switch channels if data fields are wide enough */
	if (PX4IO_RC_INPUT_CHANNELS > 17 && chancount > 15) {
		chancount = 18;

		/* channel 17 (index 16) */
		values[16] = (_frame[SBUS_FLAGS_BYTE] & (1 << 0)) * 1000 + 998;
		/* channel 18 (index 17) */
		values[17] = (_frame[SBUS_FLAGS_BYTE] & (1 << 1)) * 1000 + 998;
	}

	/* note the number of channels decoded */
	*num_values = chancount;
#endif
    
	/* decode and handle failsafe and frame-lost flags */
	if (_frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
		/* report that we failed to read anything valid off the receiver */
		_failsafe = true;
		_frame_drop = true;

	} else if (_frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
		/* set a special warning flag
		 *
		 * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
		 * condition as fail-safe greatly reduces the reliability and range of the radio link,
		 * e.g. by prematurely issueing return-to-launch!!! */

		_failsafe = false;
		_frame_drop = true;

	} else {
		_failsafe = false;
		_frame_drop = false;
	}
    
	_sbus_ringbuffer->force(&rc_report);
    
    return;
}

s32 sbus::read(u8 *buf, u32 size)
{
	u32 count = size / sizeof(struct rc_input_values);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if no data, error (we could block here) */
	if (_sbus_ringbuffer->empty())
		return -EAGAIN;

	//perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	struct rc_input_values *rc_report = reinterpret_cast<struct rc_input_values *>(buf);
	int transferred = 0;
	while (count--) {
		if (!_sbus_ringbuffer->get(rc_report))
			break;
		transferred++;
		rc_report++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(struct rc_input_values));
}

void sbus::run(void *parg)
{
	for (;;)
	{
        sbus::measure();
        msleep(2);
	}
}

}

/***********************************************************************
** End of file
***********************************************************************/
