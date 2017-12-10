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

/* ��ӡ��ʱ�����Ϣ*/
#define ERR(fmt, ...) print(3, ("[ERR] --%s--(%d)-<%s>: " fmt), \
    __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define WRN(fmt, ...) print(2, ("[WRN]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)
#define INF(fmt, ...) print(1, ("[INF]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)
#define DBG(fmt, ...) print(0, ("[DBG]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)

#define MIN(a, b)                           ((a) > (b) ? (b) : (a))
#define MAX(a, b)                           ((a) < (b) ? (b) : (a))
#define CEIL_DIV(a, b)                      (((a) + (b) - 1) / (b))	// ��������£
#define CEIL_ALIGN(a, b)                    (CEIL_DIV((a), (b)) * (b))
#define ARRAYSIZE(arr)                      (sizeof(arr) / sizeof((arr)[0]))
#define OFFSETOF(_type, _member)	        ((u32) (&((_type *)0)->_member))
#define container_of(_ptr, _type, _member)	((_type *)((char *)_ptr - OFFSETOF(_type, _member)));
#define UNREFERENCED_PARAMETER(p)           { (p) = (p); }

// map�ڴ�ֲ��ļ��������˳�����ص��ڴ�ʱ�ķֲ����������������˶�ջ�Լ����������������ĵ�ַ�������Ҫ��Ϣ
//#define ASSERT(_expr)			((_expr) ? NULL : ERR("Assert failure.\n"))
#define ASSERT(_expr)			if (!(_expr)) {ERR("Assert failure.\n");} 
	// ERR("ASSERT.\n")
	// reset    // ��������ֱ������
	// while(1) // �����ڴ˴�����������鿴�����,�������ڵ��ԣ�����ʹ�� */

#define CAPTURE_ERR()						while(1)// ͨ�ô��󲶻�


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


//			 ��       ����      �̳�
// public 	 �ɷ���   �ɷ���	 �ɼ̳�
//
// protected �ɷ���   ���ɷ���  �ɼ̳�
//
// private   �ɷ���   ���ɷ���  ���ɼ̳�

/***********************************************************************
** End of file
***********************************************************************/