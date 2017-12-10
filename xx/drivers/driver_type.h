/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/08/17
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once

#define LD_OK 	        (0)
#define LD_ERROR 	    (-1)

typedef u32 	mode_t;

typedef u32		size_t;
typedef s32		ssize_t;

typedef u32     blkcnt_t;
typedef s32   	off_t;
//typedef s32    	fpos_t;

typedef s64   	off64_t;
typedef s64   	fpos64_t;

/* The number of poll descriptors (required by poll() specification */
typedef u32 nfds_t;

/* In the standard poll() definition, the size of the event set is 'short'.
 * Here we pick the smallest storage element that will contain all of the
 * poll events.
 */
typedef u8 pollevent_t;

/***********************************************************************
** End of file
***********************************************************************/
