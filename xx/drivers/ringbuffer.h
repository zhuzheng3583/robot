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

namespace driver {

class ringbuffer
{
public:
	ringbuffer(u32 num_items, u32 item_size);
	~ringbuffer(void);

	/**
	 * Put an item into the buffer.
	 *
	 * @param val		Item to put
	 * @return		true if the item was put, false if the buffer is full
	 */
	bool put(const void *val, u32 val_size = 0);

	bool put(s8 val)	{ return put(&val, sizeof(val)); }
	bool put(u8 val)	{ return put(&val, sizeof(val)); }
	bool put(s16 val)	{ return put(&val, sizeof(val)); }
	bool put(u16 val)	{ return put(&val, sizeof(val)); }
	bool put(s32 val)	{ return put(&val, sizeof(val)); }
	bool put(u32 val)	{ return put(&val, sizeof(val)); }
	bool put(s64 val)	{ return put(&val, sizeof(val)); }
	bool put(u64 val)	{ return put(&val, sizeof(val)); }
	bool put(f32 val) 	{ return put(&val, sizeof(val)); }
	bool put(f64 val)	{ return put(&val, sizeof(val)); }


	/**
	 * Force an item into the buffer, discarding an older item if there is not space.
	 *
	 * @param val		Item to put
	 * @return		true if an item was discarded to make space
	 */
	bool force(const void *val, u32 val_size = 0);

	bool force(s8 val)	{ return force(&val, sizeof(val)); }
	bool force(u8 val) 	{ return force(&val, sizeof(val)); }
	bool force(s16 val) { return force(&val, sizeof(val)); }
	bool force(u16 val)	{ return force(&val, sizeof(val)); }
	bool force(s32 val) { return force(&val, sizeof(val)); }
	bool force(u32 val)	{ return force(&val, sizeof(val)); }
	bool force(s64 val) { return force(&val, sizeof(val)); }
	bool force(u64 val)	{ return force(&val, sizeof(val)); }
	bool force(f32 val)	{ return force(&val, sizeof(val)); }
	bool force(f64 val)	{ return force(&val, sizeof(val)); }


	/**
	 * Get an item from the buffer.
	 *
	 * @param val		Item that was gotten
	 * @return		true if an item was got, false if the buffer was empty.
	 */
	bool get(void *val, u32 val_size = 0);

	bool get(s8 &val)	{ return get(&val, sizeof(val)); }
	bool get(u8 &val)	{ return get(&val, sizeof(val)); }
	bool get(s16 &val)	{ return get(&val, sizeof(val)); }
	bool get(u16 &val) { return get(&val, sizeof(val)); }
	bool get(s32 &val)	{ return get(&val, sizeof(val)); }
	bool get(u32 &val) { return get(&val, sizeof(val)); }
	bool get(s64 &val)	{ return get(&val, sizeof(val)); }
	bool get(u64 &val) { return get(&val, sizeof(val)); }
	bool get(f32 &val)	{ return get(&val, sizeof(val)); }
	bool get(f64 &val)	{ return get(&val, sizeof(val)); }


	/*
	 * Get the number of slots free in the buffer.
	 *
	 * @return		The number of items that can be put into the buffer before
	 *			it becomes full.
	 */
	u32 space(void);

	/*
	 * Get the number of items in the buffer.
	 *
	 * @return		The number of items that can be got from the buffer before
	 *			it becomes empty.
	 */
	u32 count(void);

	/*
	 * Returns true if the buffer is empty.
	 */
	bool empty(void);

	/*
	 * Returns true if the buffer is full.
	 */
	bool full(void);

	/*
	 * Returns the capacity of the buffer, or zero if the buffer could
	 * not be allocated.
	 */
	u32 size(void);

	/*
	 * Empties the buffer.
	 */
	void flush(void);

	/*
	 * resize the buffer. This is unsafe to be called while
	 * a producer or consuming is running. Caller is responsible
	 * for any locking needed
	 *
	 * @param new_size	new size for buffer
	 * @return		true if the resize succeeds, false if
	 * 			not (allocation error)
	 */
	bool resize(unsigned new_size);

	/*
	 * printf() some info on the buffer
	 */
	void print_info(const char *name);

private:
	u32				_num_items;
	const u32		_item_size;
	s8				*_buf;
	volatile u32	_head;	/**< insertion point in _item_size units */
	volatile u32	_tail;	/**< removal point in _item_size units */

	u32 _next(u32 index);
};

}
/***********************************************************************
** End of file
***********************************************************************/


