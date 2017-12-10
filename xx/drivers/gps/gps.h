/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/08/02
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "device.h"
#include "dma.h"
#include "uart.h"


//GPS NMEA-0183Э����Ҫ�����ṹ�嶨��
//������Ϣ
__packed typedef struct
{
 	u8  num;		//���Ǳ��
	u8  eledeg;	//��������
	u16 azideg;	//���Ƿ�λ��
	u8  sn;		//�����
}nmea_slmsg;
//UTCʱ����Ϣ
__packed typedef struct
{
 	u16 year;	//���
	u8  month;	//�·�
	u8  date;	//����
	u8  hour; 	//Сʱ
	u8  min; 	//����
	u8  sec; 	//����
}nmea_utc_time;
//NMEA 0183 Э����������ݴ�Žṹ��
__packed typedef struct
{
 	u8 svnum;					//�ɼ�������
	nmea_slmsg slmsg[12];		//���12������
	nmea_utc_time utc;			//UTCʱ��
	u32 latitude;				//γ�� ������100000��,ʵ��Ҫ����100000
	u8 nshemi;					//��γ/��γ,N:��γ;S:��γ
	u32 longitude;			    //���� ������100000��,ʵ��Ҫ����100000
	u8 ewhemi;					//����/����,E:����;W:����
	u8 gpssta;					//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.
 	u8 posslnum;				//���ڶ�λ��������,0~12.
 	u8 possl[12];				//���ڶ�λ�����Ǳ��
	u8 fixmode;					//��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ
	u16 pdop;					//λ�þ������� 0~500,��Ӧʵ��ֵ0~50.0
	u16 hdop;					//ˮƽ�������� 0~500,��Ӧʵ��ֵ0~50.0
	u16 vdop;					//��ֱ�������� 0~500,��Ӧʵ��ֵ0~50.0

	int altitude;			 	//���θ߶�,�Ŵ���10��,ʵ�ʳ���10.��λ:0.1m
	u16 speed;					//��������,�Ŵ���1000��,ʵ�ʳ���10.��λ:0.001����/Сʱ
}nmea_msg;


namespace driver {

class gps : public device
{
public:
    gps(PCSTR name, s32 id);
    ~gps(void);

public:
	uart *_uart;

public:
    s32 probe(uart *puart);
    s32 remove(void);


public:
	s32 init(void);
	s32 read_raw(void);

public:
    u8 comma_pos(u8 *buf, u8 cx);
    u32 NMEA_Pow(u8 m, u8 n);
    int str2num(u8 *buf, u8*dx);
    void GPGSV_parse(nmea_msg *gpsx, u8 *buf);
    void GPGGA_parse(nmea_msg *gpsx, u8 *buf);
    void GPGSA_parse(nmea_msg *gpsx, u8 *buf);
    void GPRMC_parse(nmea_msg *gpsx,u8 *buf);
    void GPVTG_parse(nmea_msg *gpsx, u8 *buf);
    void parse(nmea_msg *gpsx, u8 *buf);
    void ublox_checksum(u8 *buf, u16 len, u8* cka, u8*ckb);

};

}


/***********************************************************************
** End of file
***********************************************************************/


