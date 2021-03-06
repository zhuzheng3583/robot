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

typedef signed char                         s8;
typedef unsigned char                       u8;
typedef signed short                        s16;
typedef unsigned short	                   	u16;
typedef signed int                          s32;
typedef unsigned int                        u32;
typedef signed long long                    s64;
typedef unsigned long long                  u64;

typedef float                               f32;
typedef double                              f64;

typedef int                                 BOOL;
typedef void *                              HANDLE;
typedef const char*                         PCSTR;

#if !defined(true)
#define true                                (1)
#define false                               (0)
#endif

#ifndef NULL
#define NULL                                0
#endif

/***********************************************************************
** End of file
***********************************************************************/