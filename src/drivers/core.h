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
#include "robot_type.h"
#include "robot_misc.h"
#include "device.h"

#define time_after(a,b)		((s32)(b) - (s32)(a) < 0)
#define time_before(a,b)	time_after(b, a)
#define time_after_eq(a,b) 	((s32)(a) - (s32)(b) >= 0)
#define time_before_eq(a,b)	time_after_eq(b, a)

/*
 * ʱ��̽���������ڲ���ָ����ʼλ���е�ָ��ĩβλ��֮�������ĵ�ʱ��
 * note ʱ���Ϊ32λ����������168MHz�£�������� = (0xFFFFFFFF+1) / 168M = 25.6s
 * 		��˲�������������t1 < t0,�⽫���²⵽��ʱ������Եĳ���
 * TODO:ʹ��64λʱ���
 */
#define START_PROFILE()                     { u64 __fc = driver::core::get_cpu_freq();              \
                                              u64 __t0 = driver::core::get_timestamp();

#define STOP_PROFILE(title)                   u64 __t1 = driver::core::get_timestamp();              \
                                              u64 __t  = 1000000ul * (__t1 - __t0) / __fc;          \
                                              INF("PROFILE[%s] = %d us.\n", (title), ((u32)__t)); }

/*
 * �豸��ʱ�����궨��
 */
#define wait_condition_count(_condition, _count, _retaddr)                  \
    do														        \
    {														        \
        *(s32 *)(_retaddr) = 0;									        \
        u32 cnt = _count;										        \
        for(; (_condition) == 0; cnt--)					                    \
        {															\
            if(cnt == 0) {										\
                *(s32 *)(_retaddr) = -1;							\
                break;												\
            }														\
        }															\
    }while(0)


#define wait_condition_us(_condition, _timeoutus, _retaddr)                 \
	do																\
	{																\
		*(s32 *)(_retaddr) = 0;										\
		u32 us = _timeoutus;									        \
		for(; (_condition) == 0; us--)					\
		{															\
			if(us == 0) {									\
				*(s32 *)(_retaddr) = -1;							\
				break;												\
			}														\
			driver::core::udelay(1);								\
		}															\
	}while(0)

#define wait_condition_ms(_condition, _timeoutms, _retaddr)	\
	do																\
	{																\
		*(s32 *)(_retaddr) = 0;										\
		u32 ms = _timeoutms;									\
		for(; (_condition) == 0; ms--)					\
		{															\
			if(ms == 0) {									\
				*(s32 *)(_retaddr) = -1;							\
				break;												\
			}														\
			driver::core::mdelay(1);								\
		}															\
	}while(0)


namespace driver {

typedef  u32    timestamp_t;

class core
{
public:
	core(void);
	~core(void);

public:
    static u32 s_freq_mhz;
    static u32 s_freq_khz;

public:
    static s32 init(void);
    static void mdelay(u32 ms);
    static void udelay(u32 us);

    static void system_clock_config(void);
    static void dwt_config(void);

    inline static u32 get_cpu_freq(void);
    static timestamp_t get_timestamp(void);
    inline static timestamp_t get_elapsed_timestamp(const volatile timestamp_t *then);
    inline static u32 convert_timestamp_to_us(u32 timestamp);
    inline static u32 convert_timestamp_to_ms(u32 timestamp);
    static timestamp_t convert_us_to_timestamp(u32 us);
	static timestamp_t convert_ms_to_timestamp(u32 ms);
    static void self_test(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/


