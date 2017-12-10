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
#include "circbuf.h"

#include <string.h>

#define 	POWEROF2(n) 	(((n) & ((n) - 1)) == 0)

namespace driver {

circbuf::circbuf(s32 size)
{
	if (POWEROF2(size)) {
		
	} else {
		CAPTURE_ERR();
	}
	_pbuf = new u8[size];
	_size = size;
	_head = 0;
	_tail = 0;
}

circbuf::~circbuf(void)
{
	delete[] _pbuf;
	_size = 0;
	_head = 0;
	_tail = 0;
}

s32 circbuf::mem_producer(u8 *pmem, s32 count)
{
	// bytes available to write
	s32 available = CIRC_SPACE(_head, _tail, _size);
	if (available < count) {
		return 0;
	}

	//int n = CIRC_SPACE_TO_END(_head, _tail, _size);	// bytes to end of the buffer
    int end = (_size) - 1 - (_head);
    int n = (end + (_tail)) & ((_size)-1);
    n = (n <= end ? n : end+1);
          
    if (n < count) {
		// message goes over end of the buffer
		memcpy(&(_pbuf[_head]), pmem, n);
		_head = 0;
	} else {
		n = 0;
	}

	// now: n = bytes already written
	s32 p = count - n;	// number of bytes to write
	memcpy(&(_pbuf[_head]), &(pmem[n]), p);
	_head = (_head + p) % _size;

	return count;
}

s32 circbuf::mem_consumer(u8 *pmem, s32 count)
{
	// bytes available to read
	s32 available = CIRC_CNT(_head, _tail, _size);
	if (available < count) {
		return 0;
	}

	//int n = CIRC_CNT_TO_END(_head, _tail, _size);	// bytes to end of the buffer
    int end = (_size) - (_tail);
    int n = ((_head) + end) & ((_size)-1);
    n = (n < end ? n : end);
	if (n < count) {
		// message goes over end of the buffer
		memcpy(pmem, &(_pbuf[_tail]), n);
		_tail = 0;
	} else {
		n = 0;
	}

	// now: n = bytes already read
	s32 p = count - n;	// number of bytes to write
	memcpy(&(pmem[n]), &(_pbuf[_tail]), p);
	_tail = (_tail + p) % _size;

	return count;
}

s32 circbuf::dev_consumer(device *dev, s32 count)
{
	// bytes available to read
	s32 available = CIRC_CNT(_head, _tail, _size);
	if (available < count) {
		return 0;
	}

	//int n = CIRC_CNT_TO_END(_head, _tail, _size);	// bytes to end of the buffer
    int end = (_size) - (_tail);
    int n = ((_head) + end) & ((_size)-1);
    n = (n < end ? n : end);
	if (n < count) {
		// message goes over end of the buffer
		dev->write(&(_pbuf[_tail]), n);
		_tail = 0;
	} else {
		n = 0;
	}

	// now: n = bytes already read
	s32 p = count - n;	// number of bytes to write
	dev->write(&(_pbuf[_tail]), p);
	_tail = (_tail + p) % _size;

	return count;
}

s32 circbuf::dev_producer(device *dev, s32 count)
{
	// bytes available to write
	s32 available = CIRC_SPACE(_head, _tail, _size);
	if (available < count) {
		return 0;
	}

	//int n = CIRC_SPACE_TO_END(_head, _tail, _size);	// bytes to end of the buffer
    int end = (_size) - 1 - (_head);
    int n = (end + (_tail)) & ((_size)-1);
    n = (n <= end ? n : end+1);
	if (n < count) {
		// message goes over end of the buffer
		dev->read(&(_pbuf[_head]), n);
		_head = 0;
	} else {
		n = 0;
	}

	// now: n = bytes already written
	s32 p = count - n;	// number of bytes to write
	dev->read(&(_pbuf[_head]), p);
	_head = (_head + p) % _size;

	return count;
}

}

/***********************************************************************
** End of file
***********************************************************************/


