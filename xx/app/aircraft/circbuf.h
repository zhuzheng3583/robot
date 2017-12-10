/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/

#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "device.h"

//#define CIRC_CNT(head,tail,size) ((head) >= (tail) ? (head)-(tail) : (size)-(tail)+(head))
//#define CIRC_SPACE(head,tail,size) ((size-1) - CIRC_CNT(((head)),(tail),(size)))

/* Return count in buffer.  */
// size必须为2^n
#define CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))

/* Return space available, 0..size-1.  We always leave one free char
   as a completely full buffer has head == tail, which is the same as
   empty.  */
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))

/* Return count up to the end of the buffer.  Carefully avoid
   accessing head and tail more than once, so they can change
   underneath us without returning inconsistent results.  */
// 返回连续可用的数据字节数
#define CIRC_CNT_TO_END(head,tail,size) \
	({int end = (size) - (tail); \
	  int n = ((head) + end) & ((size)-1); \
	  n < end ? n : end;})

/* Return space available up to the end of the buffer.  */
// 返回连续可用的空间字节数
#define CIRC_SPACE_TO_END(head,tail,size) \
	({int end = (size) - 1 - (head); \
	  int n = (end + (tail)) & ((size)-1); \
	  n <= end ? n : end+1;})


namespace driver {

class circbuf
{
public:
	circbuf(s32 size);
	~circbuf(void);

public:
	u8*	_pbuf;				// 缓冲区首地址
	s32	_size;				// 缓冲区大小
	s32	_head;				// 缓冲区头指针
	s32	_tail;				// 缓冲区尾指针

public:
	//mem==>circbuf
	s32 mem_producer(u8 *pmem, s32 count);
	//mem<==circbuf
	s32 mem_consumer(u8 *pmem, s32 count);
	//circbuf==>device
	s32 dev_consumer(device *dev, s32 count);
	//circbuf<==device
	s32 dev_producer(device *dev, s32 count);

	u32 dev_consumer_to_empty(device *dev)	{ return circbuf::dev_consumer(dev, _size); }
	u32 dev_producer_to_full(device *dev)	{ return circbuf::dev_producer(dev, _size); }
	
public:
	u8* get_addr(void)			{ return _pbuf; }
	s32 get_total_size(void)	{ return _size; }
	s32 get_free_size(void)		{ return (CIRC_SPACE(_head, _tail, _size)); }
	s32 get_used_size(void)		{ return (CIRC_CNT(_head, _tail, _size)); }
	s32 get_head_pos(void)		{ return _head; }
	s32 get_tail_pos(void)		{ return _tail; }
	void reset_circbuf(void)	{ _head = _tail = 0; }
};

}







/***********************************************************************
** End of file
***********************************************************************/

