/*******************************Copyright (c)***************************
** 
** Porject name:	robot
** Created by:		
** Created date:	
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "robot_type.h"

#ifdef __cplusplus
extern "C" {
#endif
  
int	print(s32 level, const char* fmt, ...);

#ifdef __cplusplus
}
#endif

/* 打印出时间戳信息*/
#define ERR(fmt, ...) print(3, ("[ERR] --%s--(%d)-<%s>: " fmt), \
    __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define WRN(fmt, ...) print(2, ("[WRN]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)
#define INF(fmt, ...) print(1, ("[INF]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)
#define DBG(fmt, ...) print(0, ("[DBG]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)

#define MIN(a, b)                           ((a) > (b) ? (b) : (a))
#define MAX(a, b)                           ((a) < (b) ? (b) : (a))
#define CEIL_DIV(a, b)                      (((a) + (b) - 1) / (b))	// 向正方向靠拢
#define CEIL_ALIGN(a, b)                    (CEIL_DIV((a), (b)) * (b))
#define ARRAYSIZE(arr)                      (sizeof(arr) / sizeof((arr)[0]))
#define OFFSETOF(_type, _member)	        ((u32) (&((_type *)0)->_member))
#define container_of(_ptr, _type, _member)	((_type *)((char *)_ptr - OFFSETOF(_type, _member)));
#define UNREFERENCED_PARAMETER(p)           { (p) = (p); }

// map内存分布文件：描述了程序加载到内存时的分布情况，因此它包含了堆栈以及各个变量、函数的地址分配等重要信息
//#define ASSERT(_expr)			((_expr) ? NULL : ERR("Assert failure.\n"))
#define ASSERT(_expr)			if (!(_expr)) {ERR("Assert failure.\n");} 
	// ERR("ASSERT.\n")
	// reset    // 不理会错误直接重启
	// while(1) // 卡死在此处方便仿真器查看出错点,仅仅用于调试，慎重使用 */

#define CAPTURE_ERR()						while(1)// 通用错误捕获


#define BYTE_SIZE							(1)
#define HALF_WORD_SIZE						(2)
#define WORD_SIZE							(4)

#if defined(WIN32)
	#define	TARGET_WINDOWS
#else
	#define	TARGET_STM32
#endif

#if !defined(__cplusplus)
	#define inline							__inline
#endif


//			 类       对象      继承
// public 	 可访问   可访问	 可继承
//
// protected 可访问   不可访问  可继承
//
// private   可访问   不可访问  不可继承

/***********************************************************************
** End of file
***********************************************************************/