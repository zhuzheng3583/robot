/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	    zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "ringbuffer.h"

namespace driver {

ringbuffer::ringbuffer(u32 num_items, u32 item_size) :
	_num_items(num_items),
	_item_size(item_size),
	_buf(new s8[(_num_items+1) * item_size]),
 	_head(_num_items),
 	_tail(_num_items)
{

}

ringbuffer::~ringbuffer(void)
{
	if (_buf != NULL)
		delete[] _buf;
}

u32 ringbuffer::_next(u32 index)
{
	return (0 == index) ? _num_items : (index - 1);
}

bool ringbuffer::empty(void)
{
	return _tail == _head;
}

bool ringbuffer::full(void)
{
	return _next(_head) == _tail;
}

u32 ringbuffer::size(void)
{
	return (_buf != NULL) ? _num_items : 0;
}

void ringbuffer::flush(void)
{
	while (!empty())
		get(NULL);
}

bool ringbuffer::put(const void *val, u32 val_size)
{
	u32 next = _next(_head);
	if (next != _tail) {
		if ((val_size == 0) || (val_size > _item_size))
			val_size = _item_size;
		memcpy(&_buf[_head * _item_size], val, val_size);
		_head = next;
		return true;
	} else {
		return false;
	}
}

bool ringbuffer::force(const void *val, u32 val_size)
{
	bool overwrote = false;

	for (;;) {
		if (put(val, val_size))
			break;
		get(NULL);
		overwrote = true;
	}
	return overwrote;
}

bool ringbuffer::get(void *val, u32 val_size)
{
	if (_tail != _head) {
		u32 candidate;
		u32 next;

		if ((val_size == 0) || (val_size > _item_size))
			val_size = _item_size;

		do {
			/* decide which element we think we're going to read */
			candidate = _tail;

			/* and what the corresponding next index will be */
			next = _next(candidate);

			/* go ahead and read from this index */
			if (val != NULL)
				memcpy(val, &_buf[candidate * _item_size], val_size);

			//disable irq
			/* if the tail pointer didn't change, we got our item */
			if (_tail == candidate) {
				_tail = next;
				break;
			} else {
				continue;
			}
			//enable irq
		} while(1);//while (!__sync_bool_compare_and_swap(&_tail, candidate, next));


		return true;
	} else {
		return false;
	}
}

u32 ringbuffer::space(void)
{
	unsigned tail, head;

	/*
	 * Make a copy of the head/tail pointers in a fashion that
	 * may err on the side of under-estimating the free space
	 * in the buffer in the case that the buffer is being updated
	 * asynchronously with our check.
	 * If the head pointer changes (reducing space) while copying,
	 * re-try the copy.
	 */
	do {
		head = _head;
		tail = _tail;
	} while (head != _head);

	return (tail >= head) ? (_num_items - (tail - head)) : (head - tail - 1);
}

u32 ringbuffer::count(void)
{
	/*
	 * Note that due to the conservative nature of space(), this may
	 * over-estimate the number of items in the buffer.
	 */
	return _num_items - space();
}

bool ringbuffer::resize(unsigned new_size)
{
	s8 *old_buffer;
	s8 *new_buffer = new s8 [(new_size+1) * _item_size];
	if (new_buffer == NULL) {
		return false;
	}
	old_buffer = _buf;
	_buf = new_buffer;
	_num_items = new_size;
	_head = new_size;
	_tail = new_size;
	delete[] old_buffer;
	return true;
}

void ringbuffer::print_info(const char *name)
{
	INF("%s	%u/%u (%u/%u @ %p)\n",
		name,
		_num_items,
		_num_items * _item_size,
		_head,
		_tail,
		_buf);
}

}
/***********************************************************************
** End of file
***********************************************************************/



