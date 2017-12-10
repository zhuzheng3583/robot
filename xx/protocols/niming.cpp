/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#if 0//NIMING_V26
#include "niming.h"

#include "misc_usr.h"
#include "ifile.h"


/**
 * 传送数据给匿名四轴上位机软件(V2.6版本)
 * fun:	功能字. 0XA0~0XAF
 * data:数据缓存区,最多28字节
 * len:	data区有效数据个数
 */
static void niming_report(u8 fun, u8 *data, u8 len)
{
	if(len>28) return;						// 最多28字节数据

	u8 i;
	u8 send_buf[32];

	send_buf[0]		= 0X88;					// 帧头
	send_buf[1]		= fun;					// 功能字
	send_buf[2]		= len;					// 数据长度
	send_buf[len+3]	= 0;					// 校验数置零

	for(i = 0; i < len; i++) {
		send_buf[3+i] = data[i];			// 复制数据
	}
	for(i=0;i<len+3;i++) {
		send_buf[len+3] += send_buf[i];		// 计算校验和
	}

	write(get_fd()->uart3, send_buf, len+4);
}

/**
 * 发送加速度传感器数据和陀螺仪数据
 * aacx,aacy,aacz:		x,y,z三个方向上面的加速度值
 * gyrox,gyroy,gyroz:	x,y,z三个方向上面的陀螺仪值
 */
void mpu_send_data(s16 aacx, s16 aacy, s16 aacz, s16 gyrox, s16 gyroy, s16 gyroz)
{
#if 0
	u8 tbuf[12];
  	tbuf[0]  = (aacx>>8)&0XFF;
  	tbuf[1]	 = aacx&0XFF;
  	tbuf[2]	 = (aacy>>8)&0XFF;
  	tbuf[3]	 = aacy&0XFF;
  	tbuf[4]	 = (aacz>>8)&0XFF;
  	tbuf[5]	 = aacz&0XFF;
  	tbuf[6]	 = (gyrox>>8)&0XFF;
  	tbuf[7]	 = gyrox&0XFF;
  	tbuf[8]	 = (gyroy>>8)&0XFF;
  	tbuf[9]	 = gyroy&0XFF;
	tbuf[10] = (gyroz>>8)&0XFF;
	tbuf[11] = gyroz&0XFF;
	niming_report(0XA1, tbuf, 12);			// 自定义帧,0XA1
#else
	u8 tbuf[12];
  	tbuf[0]  = (aacx>>8)&0XFF;
  	tbuf[1]	 = aacx&0XFF;
  	tbuf[2]	 = (aacy>>8)&0XFF;
  	tbuf[3]	 = aacy&0XFF;
  	tbuf[4]	 = (aacz>>8)&0XFF;
  	tbuf[5]	 = aacz&0XFF;
  	tbuf[6]	 = (gyrox>>8)&0XFF;
  	tbuf[7]	 = gyrox&0XFF;
  	tbuf[8]	 = (gyroy>>8)&0XFF;
  	tbuf[9]	 = gyroy&0XFF;
	tbuf[10] = (gyroz>>8)&0XFF;
	tbuf[11] = gyroz&0XFF;
	niming_report(0XA1, &tbuf[0], 2);			// 自定义帧,0XA1
	niming_report(0XA2, &tbuf[2], 2);			// 自定义帧,0XA2
	niming_report(0XA3, &tbuf[4], 2);			// 自定义帧,0XA3
	niming_report(0XA4, &tbuf[6], 2);			// 自定义帧,0XA4
	niming_report(0XA5, &tbuf[8], 2);			// 自定义帧,0XA5
	niming_report(0XA6, &tbuf[10], 2);			// 自定义帧,0XA6
#endif
}

/*
void multi_channel_send_data(U8 *buf, U8 count, U8 ch_start, U8 ch_count, U8 step)
{
	count
	int i = 0;
	for (i = 0; i < )
	{

	}
}
*/
/**
 * 发送解算后的姿态数据
 * aacx,aacy,aacz:		x,y,z三个方向上面的加速度值
 * gyrox,gyroy,gyroz:	x,y,z三个方向上面的陀螺仪值
 * roll: 	横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
 * pitch:	俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
 * yaw: 	航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
 */
void imu_send_data(s16 aacx,s16 aacy,s16 aacz,s16 gyrox,s16 gyroy,s16 gyroz,s16 roll,s16 pitch,s16 yaw)
{
	u8 i;
	u8 tbuf[28];

	for(i = 0; i < 28; i++) {
		tbuf[i] = 0;// 清0
	}

	tbuf[0]  = (aacx>>8)&0XFF;
	tbuf[1]	 = aacx&0XFF;
	tbuf[2]	 = (aacy>>8)&0XFF;
	tbuf[3]	 = aacy&0XFF;
	tbuf[4]	 = (aacz>>8)&0XFF;
	tbuf[5]	 = aacz&0XFF;
	tbuf[6]	 = (gyrox>>8)&0XFF;
	tbuf[7]	 = gyrox&0XFF;
	tbuf[8]	 = (gyroy>>8)&0XFF;
	tbuf[9]	 = gyroy&0XFF;
	tbuf[10] = (gyroz>>8)&0XFF;
	tbuf[11] = gyroz&0XFF;
	tbuf[18] = (roll>>8)&0XFF;
	tbuf[19] = roll&0XFF;
	tbuf[20] = (pitch>>8)&0XFF;
	tbuf[21] = pitch&0XFF;
	tbuf[22] = (yaw>>8)&0XFF;
	tbuf[23] = yaw&0XFF;
	niming_report(0XAF, tbuf, 28);				// 飞控显示帧,0XAF
}
#endif


/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/04/08
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "niming.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

namespace app {

niming::niming(void)
{

}

niming::~niming(void)
{

}

void niming::attach(uart *puart)
{
	_uart = puart;
	_uart->open();
}

// 1：发送基本信息（姿态、锁定状态）
void niming::report_status(struct vehicle_attitude_s *att)
{
	u8 cnt = 0;
	u8 data[50];
	s16 temp = 0;
	s32 temp2 = 0;
	u8 sum = 0;

	data[cnt++] = 0xAA;
	data[cnt++] = 0xAA;
	data[cnt++] = 0x01;
	data[cnt++] = 0;

	temp = att->roll * 100;
	data[cnt++] = BYTE1(temp);
	data[cnt++] = BYTE0(temp);
	temp = att->pitch * 100;
	data[cnt++] = BYTE1(temp);
	data[cnt++] = BYTE0(temp);
	temp = att->yaw * 100;
	data[cnt++] = BYTE1(temp);
	data[cnt++] = BYTE0(temp);

	temp2 = 4;//atti->altitude;
	data[cnt++] = BYTE3(temp2);
	data[cnt++] = BYTE2(temp2);
	data[cnt++] = BYTE1(temp2);
	data[cnt++] = BYTE0(temp2);

	//if(Rc_C.ARMED==0)		data[cnt++]=0xA0;
	//else if(Rc_C.ARMED==1)		data[cnt++]=0xA1;
	data[cnt++]=0xA1;

	data[3] = cnt - 4;

	for(u8 i = 0; i < cnt; i++) {
		sum += data[i];
	}

	data[cnt++] = sum;

	_uart->write(data, cnt);
}

// 2：发送传感器数据
void niming::report_sensor(bool type_raw, struct accel_report *accel,
    struct gyro_report *gyro, struct mag_report *mag)
{
	u8 cnt = 0;
	u8 data[64];
    s16 temp = 0;
    u8 sum = 0;

	data[cnt++] = 0xAA;
	data[cnt++] = 0xAA;
	data[cnt++] = 0x02;
	data[cnt++] = 0;

    if (type_raw == true) {
        temp = (s16)accel->x_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)accel->y_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)accel->z_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);

        temp = (s16)gyro->x_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)gyro->y_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)gyro->z_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);

        temp = (s16)mag->x_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)mag->y_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)mag->z_raw;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);

    } else {
        temp = (s16)accel->x;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)accel->y;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)accel->z;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);

        temp = (s16)gyro->x;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)gyro->y;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)gyro->z;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);

        temp = (s16)mag->x;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)mag->y;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
        temp = (s16)mag->z;
        data[cnt++] = BYTE1(temp);
        data[cnt++] = BYTE0(temp);
    }

	data[3] = cnt - 4;

	for(u8 i = 0; i < cnt; i++) {
		sum += data[i];
	}
	data[cnt++] = sum;

	_uart->write(data, cnt);
}

#if 0
void niming::report_rc(data_rc_t *rc)
{
	u8 cnt = 0;
	u8 data[32];
	u8 sum = 0;

	data[cnt++] = 0xAA;
	data[cnt++] = 0xAA;
	data[cnt++] = 0x03;
	data[cnt++] = 0;
	data[cnt++] = BYTE1(rc->throttle);
	data[cnt++] = BYTE0(rc->throttle);
	data[cnt++] = BYTE1(rc->yaw);
	data[cnt++] = BYTE0(rc->yaw);
	data[cnt++] = BYTE1(rc->roll);
	data[cnt++] = BYTE0(rc->roll);
	data[cnt++] = BYTE1(rc->pitch);
	data[cnt++] = BYTE0(rc->pitch);
	data[cnt++] = BYTE1(rc->aux1);
	data[cnt++] = BYTE0(rc->aux1);
	data[cnt++] = BYTE1(rc->aux2);
	data[cnt++] = BYTE0(rc->aux2);
	data[cnt++] = BYTE1(rc->aux3);
	data[cnt++] = BYTE0(rc->aux3);
	data[cnt++] = BYTE1(rc->aux4);
	data[cnt++] = BYTE0(rc->aux4);
	data[cnt++] = BYTE1(rc->aux5);
	data[cnt++] = BYTE0(rc->aux5);
	data[cnt++] = BYTE1(rc->aux6);
	data[cnt++] = BYTE0(rc->aux6);

	data[3] = cnt-4;

	for(u8 i = 0; i < cnt; i++)
		sum += data[i];

	data[cnt++]=sum;

	_uart->write(data, cnt);
}
#endif

#if 0
void Data_Send_MotoPWM(void)
{
	u8 cnt=0;
	data[cnt++]=0xAA;
	data[cnt++]=0xAA;
	data[cnt++]=0x06;
	data[cnt++]=0;
	data[cnt++]=BYTE1(Moto_PWM_1);
	data[cnt++]=BYTE0(Moto_PWM_1);
	data[cnt++]=BYTE1(Moto_PWM_2);
	data[cnt++]=BYTE0(Moto_PWM_2);
	data[cnt++]=BYTE1(Moto_PWM_3);
	data[cnt++]=BYTE0(Moto_PWM_3);
	data[cnt++]=BYTE1(Moto_PWM_4);
	data[cnt++]=BYTE0(Moto_PWM_4);
	data[cnt++]=BYTE1(Moto_PWM_5);
	data[cnt++]=BYTE0(Moto_PWM_5);
	data[cnt++]=BYTE1(Moto_PWM_6);
	data[cnt++]=BYTE0(Moto_PWM_6);
	data[cnt++]=BYTE1(Moto_PWM_7);
	data[cnt++]=BYTE0(Moto_PWM_7);
	data[cnt++]=BYTE1(Moto_PWM_8);
	data[cnt++]=BYTE0(Moto_PWM_8);

	data[3] = cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<cnt;i++)
		sum += data[i];

	data[cnt++]=sum;

#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data,cnt);
#else
	NRF_TxPacket(data,cnt);
#endif
}
void Data_Send_OFFSET(void)
{
	u8 cnt=0;
	data[cnt++]=0xAA;
	data[cnt++]=0xAA;
	data[cnt++]=0x16;
	data[cnt++]=0;
	vs16 _temp = AngleOffset_Rol*1000;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = AngleOffset_Pit*1000;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);

	data[3] = cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<cnt;i++)
		sum += data[i];

	data[cnt++]=sum;

#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data,cnt);
#else
	NRF_TxPacket(data,cnt);
#endif
}
void Data_Send_PID1(void)
{
	u8 cnt=0;
	data[cnt++]=0xAA;
	data[cnt++]=0xAA;
	data[cnt++]=0x10;
	data[cnt++]=0;

	vs16 _temp;
	_temp = PID_ROL.P * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_ROL.I * 1000;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_ROL.D * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_PIT.P * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_PIT.I * 1000;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_PIT.D * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_YAW.P * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_YAW.I * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_YAW.D * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);

	data[3] = cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<cnt;i++)
		sum += data[i];

	data[cnt++]=sum;

#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data,cnt);
#else
	NRF_TxPacket(data,cnt);
#endif
}
void Data_Send_PID2(void)
{
	u8 cnt=0;
	data[cnt++]=0xAA;
	data[cnt++]=0xAA;
	data[cnt++]=0x11;
	data[cnt++]=0;

	vs16 _temp;
	_temp = PID_ALT.P * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_ALT.I * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_ALT.D * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_POS.P * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_POS.I * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_POS.D * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_PID_1.P * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_PID_1.I * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_PID_1.D * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);

	data[3] = cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<cnt;i++)
		sum += data[i];

	data[cnt++]=sum;

#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data,cnt);
#else
	NRF_TxPacket(data,cnt);
#endif
}
void Data_Send_PID3(void)
{
	u8 cnt=0;
	data[cnt++]=0xAA;
	data[cnt++]=0xAA;
	data[cnt++]=0x12;
	data[cnt++]=0;

	vs16 _temp;
	_temp = PID_PID_2.P * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_PID_2.I * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);
	_temp = PID_PID_2.D * 100;
	data[cnt++]=BYTE1(_temp);
	data[cnt++]=BYTE0(_temp);

	data[3] = cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<cnt;i++)
		sum += data[i];

	data[cnt++]=sum;

#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data,cnt);
#else
	NRF_TxPacket(data,cnt);
#endif
}
void Data_Send_Check(u16 check)
{
	data[0]=0xAA;
	data[1]=0xAA;
	data[2]=0xF0;
	data[3]=3;
	data[4]=0xBA;

	data[5]=BYTE1(check);
	data[6]=BYTE0(check);

	u8 sum = 0;
	for(u8 i=0;i<7;i++)
		sum += data[i];

	data[7]=sum;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data,8);
#else
	NRF_TxPacket(data,8);
#endif
}

#endif
}


/***********************************************************************
** End of file
***********************************************************************/


