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

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "leader_type.h"
#include "leader.h"

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

// TODO:失控保护:面对复杂的外界环境的干扰，无论是未知的异常(比如hardfault)或者其他操作超时以及未知的错误等一概重启MCU
// 对于异常的处理：dump出ARM core寄存器，pc sp等值，以便通过map文件定位异常跳转前的下一条指令,
// 比如执行到除数为0的指令，或者空指针，或者硬件异常等，均可通过编程stm32f4xx_it.c中的异常处理程序来dump堆栈信息,
// 比如：TI sysbios中在抛出异常时打印了CPU核寄存器pc,sp等，再依据map文件便可以找到错误时的堆栈;
// 再如：Linux中oops常常在执行到空指针的时候抛出堆栈信息,
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

#define USE_UCOS3                           0
#define USE_STM32F4_DEMO                    0

//			 类       对象      继承
// public 	 可访问   可访问	 可继承
//
// protected 可访问   不可访问  可继承
//
// private   可访问   不可访问  不可继承

/***********************************************************************
** End of file
***********************************************************************/