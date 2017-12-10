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

#if 0

////////////////////////////////////////////////////////////////////////////////////////////////////
//UBLOX NEO-6M ����(���,����,���ص�)�ṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG CFG ID:0X0906 (С��ģʽ)
	u16 dlength;				//���ݳ��� 12/13
	u32 clearmask;				//�������������(1��Ч)
	u32 savemask;				//�����򱣴�����
	u32 loadmask;				//�������������
	u8  devicemask; 		  	//Ŀ������ѡ������	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	u8  cka;		 			//У��CK_A
	u8  ckb;			 		//У��CK_B
}_ublox_cfg_cfg;

//UBLOX NEO-6M ��Ϣ���ýṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG MSG ID:0X0106 (С��ģʽ)
	u16 dlength;				//���ݳ��� 8
	u8  msgclass;				//��Ϣ����(F0 ����NMEA��Ϣ��ʽ)
	u8  msgid;					//��Ϣ ID
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	u8  iicset;					//IIC���������    0,�ر�;1,ʹ��.
	u8  uart1set;				//UART1�������	   0,�ر�;1,ʹ��.
	u8  uart2set;				//UART2�������	   0,�ر�;1,ʹ��.
	u8  usbset;					//USB�������	   0,�ر�;1,ʹ��.
	u8  spiset;					//SPI�������	   0,�ر�;1,ʹ��.
	u8  ncset;					//δ֪�������	   Ĭ��Ϊ1����.
 	u8  cka;			 		//У��CK_A
	u8  ckb;			    	//У��CK_B
}_ublox_cfg_msg;

//UBLOX NEO-6M UART�˿����ýṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG PRT ID:0X0006 (С��ģʽ)
	u16 dlength;				//���ݳ��� 20
	u8  portid;					//�˿ں�,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	u8  reserved;				//����,����Ϊ0
	u16 txready;				//TX Ready��������,Ĭ��Ϊ0
	u32 mode;					//���ڹ���ģʽ����,��żУ��,ֹͣλ,�ֽڳ��ȵȵ�����.
 	u32 baudrate;				//����������
 	u16 inprotomask;		 	//����Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	u16 outprotomask;		 	//���Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	u16 reserved4; 				//����,����Ϊ0
 	u16 reserved5; 				//����,����Ϊ0
 	u8  cka;			 		//У��CK_A
	u8  ckb;			    	//У��CK_B
}_ublox_cfg_prt;

//UBLOX NEO-6M ʱ���������ýṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG TP ID:0X0706 (С��ģʽ)
	u16 dlength;				//���ݳ���
	u32 interval;				//ʱ��������,��λΪus
	u32 length;				 	//������,��λΪus
	signed char status;			//ʱ����������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
	u8 timeref;			   		//�ο�ʱ��:0,UTCʱ��;1,GPSʱ��;2,����ʱ��.
	u8 flags;					//ʱ���������ñ�־
	u8 reserved;				//����
 	signed short antdelay;	 	//������ʱ
 	signed short rfdelay;		//RF��ʱ
	signed int userdelay; 	 	//�û���ʱ
	u8 cka;						//У��CK_A
	u8 ckb;						//У��CK_B
}_ublox_cfg_tp;

//UBLOX NEO-6M ˢ���������ýṹ��
__packed typedef struct
{
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG RATE ID:0X0806 (С��ģʽ)
	u16 dlength;				//���ݳ���
	u16 measrate;				//����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
	u16 navrate;				//�������ʣ����ڣ����̶�Ϊ1
	u16 timeref;				//�ο�ʱ�䣺0=UTC Time��1=GPS Time��
 	u8  cka;					//У��CK_A
	u8  ckb;					//У��CK_B
}_ublox_cfg_rate;


/////////////////////////////////////////UBLOX ���ô���/////////////////////////////////////
//���CFG����ִ�����
//����ֵ:0,ACK�ɹ�
//       1,���ճ�ʱ����
//       2,û���ҵ�ͬ���ַ�
//       3,���յ�NACKӦ��
u8 ublox_cfg_ack_check(void)
{
	u16 len=0,i;
	u8 rval=0;
	while((USART3_RX_STA & 0X8000) == 0 && len < 100)//�ȴ����յ�Ӧ��
	{
		len++;
		delay_ms(5);
	}
	if(len < 250)   	//��ʱ����.
	{
		len = USART3_RX_STA & 0X7FFF;	//�˴ν��յ������ݳ���
		for(i=0; i < len; i++)
			if(USART3_RX_BUF[i]==0XB5)
				break;//����ͬ���ַ� 0XB5
		if(i == len)
			rval = 2;						//û���ҵ�ͬ���ַ�
		else if(USART3_RX_BUF[i+3] == 0X00)
			rval = 3;//���յ�NACKӦ��
		else
			rval = 0;	   						//���յ�ACKӦ��
	}
	else
	rval = 1;								//���ճ�ʱ����
	USART3_RX_ST A= 0;							//�������
	return rval;
}
//���ñ���
//����ǰ���ñ������ⲿEEPROM����
//����ֵ:0,ִ�гɹ�;1,ִ��ʧ��.
u8 Ublox_Cfg_Cfg_Save(void)
{
	u8 i;
	_ublox_cfg_cfg cfg;
	_ublox_cfg_cfg *cfg_cfg=&cfg;
	cfg_cfg->header=0X62B5;		//cfg header
	cfg_cfg->id=0X0906;			//cfg cfg id
	cfg_cfg->dlength=13;		//����������Ϊ13���ֽ�.
	cfg_cfg->clearmask=0;		//�������Ϊ0
	cfg_cfg->savemask=0XFFFF; 	//��������Ϊ0XFFFF
	cfg_cfg->loadmask=0; 		//��������Ϊ0
	cfg_cfg->devicemask=4; 		//������EEPROM����
	ublox_checksum((u8*)(&cfg_cfg->id),sizeof(_ublox_cfg_cfg)-4,&cfg_cfg->cka,&cfg_cfg->ckb);
	ublox_send_date((u8*)cfg_cfg,sizeof(_ublox_cfg_cfg));//�������ݸ�NEO-6M
	for(i=0;i<6;i++)if(ublox_cfg_ack_check()==0)break;		//EEPROMд����Ҫ�ȽϾ�ʱ��,���������ж϶��
	return i==6?1:0;
}
//����NMEA�����Ϣ��ʽ
//msgid:Ҫ������NMEA��Ϣ��Ŀ,���������Ĳ�����
//      00,GPGGA;01,GPGLL;02,GPGSA;
//		03,GPGSV;04,GPRMC;05,GPVTG;
//		06,GPGRS;07,GPGST;08,GPZDA;
//		09,GPGBS;0A,GPDTM;0D,GPGNS;
//uart1set:0,����ر�;1,�������.
//����ֵ:0,ִ�гɹ�;����,ִ��ʧ��.
u8 Ublox_Cfg_Msg(u8 msgid,u8 uart1set)
{
	_ublox_cfg_msg msg;
	_ublox_cfg_msg *cfg_msg=&msg;
	cfg_msg->header=0X62B5;		//cfg header
	cfg_msg->id=0X0106;			//cfg msg id
	cfg_msg->dlength=8;			//����������Ϊ8���ֽ�.
	cfg_msg->msgclass=0XF0;  	//NMEA��Ϣ
	cfg_msg->msgid=msgid; 		//Ҫ������NMEA��Ϣ��Ŀ
	cfg_msg->iicset=1; 			//Ĭ�Ͽ���
	cfg_msg->uart1set=uart1set; //��������
	cfg_msg->uart2set=1; 	 	//Ĭ�Ͽ���
	cfg_msg->usbset=1; 			//Ĭ�Ͽ���
	cfg_msg->spiset=1; 			//Ĭ�Ͽ���
	cfg_msg->ncset=1; 			//Ĭ�Ͽ���
	ublox_checksum((u8*)(&cfg_msg->id),sizeof(_ublox_cfg_msg)-4,&cfg_msg->cka,&cfg_msg->ckb);
	ublox_send_date((u8*)cfg_msg,sizeof(_ublox_cfg_msg));//�������ݸ�NEO-6M
	return ublox_cfg_ack_check();
}
//����NMEA�����Ϣ��ʽ
//baudrate:������,4800/9600/19200/38400/57600/115200/230400
//����ֵ:0,ִ�гɹ�;����,ִ��ʧ��(���ﲻ�᷵��0��)
u8 ublox_cfg_prt(u32 baudrate)
{
	_ublox_cfg_prt cfg;
	_ublox_cfg_prt *cfg_prt=&cfg;
	cfg_prt->header=0X62B5;		//cfg header
	cfg_prt->id=0X0006;			//cfg prt id
	cfg_prt->dlength=20;		//����������Ϊ20���ֽ�.
	cfg_prt->portid=1;			//��������1
	cfg_prt->reserved=0;	 	//�����ֽ�,����Ϊ0
	cfg_prt->txready=0;	 		//TX Ready����Ϊ0
	cfg_prt->mode=0X08D0; 		//8λ,1��ֹͣλ,��У��λ
	cfg_prt->baudrate=baudrate; //����������
	cfg_prt->inprotomask=0X0007;//0+1+2
	cfg_prt->outprotomask=0X0007;//0+1+2
 	cfg_prt->reserved4=0; 		//�����ֽ�,����Ϊ0
 	cfg_prt->reserved5=0; 		//�����ֽ�,����Ϊ0
	ublox_checksum((u8*)(&cfg_prt->id),sizeof(_ublox_cfg_prt)-4,&cfg_prt->cka,&cfg_prt->ckb);
	ublox_send_date((u8*)cfg_prt,sizeof(_ublox_cfg_prt));//�������ݸ�NEO-6M
	delay_ms(200);				//�ȴ��������
	usart3_init(baudrate);	//���³�ʼ������3
	return ublox_cfg_ack_check();//���ﲻ�ᷴ��0,��ΪUBLOX��������Ӧ���ڴ������³�ʼ����ʱ���Ѿ���������.
}
//����UBLOX NEO-6��ʱ���������
//interval:������(us)
//length:������(us)
//status:��������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
//����ֵ:0,���ͳɹ�;����,����ʧ��.
u8 ublox_cfg_yp(u32 interval,u32 length,signed char status)
{
	_ublox_cfg_tp tp;
	_ublox_cfg_tp *cfg_tp=&tp;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//����������Ϊ20���ֽ�.
	cfg_tp->interval=interval;	//������,us
	cfg_tp->length=length;		//������,us
	cfg_tp->status=status;	   	//ʱ����������
	cfg_tp->timeref=0;			//�ο�UTC ʱ��
	cfg_tp->flags=0;			//flagsΪ0
	cfg_tp->reserved=0;		 	//����λΪ0
	cfg_tp->antdelay=820;    	//������ʱΪ820ns
	cfg_tp->rfdelay=0;    		//RF��ʱΪ0ns
	cfg_tp->userdelay=0;    	//�û���ʱΪ0ns
	ublox_checksum((u8*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	ublox_send_date((u8*)cfg_tp,sizeof(_ublox_cfg_tp));//�������ݸ�NEO-6M
	return ublox_cfg_ack_check();
}
//����UBLOX NEO-6�ĸ�������
//measrate:����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
//reftime:�ο�ʱ�䣬0=UTC Time��1=GPS Time��һ������Ϊ1��
//����ֵ:0,���ͳɹ�;����,����ʧ��.
u8 ublox_cfg_rate(u16 measrate,u8 reftime)
{
	_ublox_cfg_rate rate;
	_ublox_cfg_rate *cfg_rate=&rate;
 	if(measrate<200)return 1;	//С��200ms��ֱ���˳�
 	cfg_rate->header=0X62B5;	//cfg header
	cfg_rate->id=0X0806;	 	//cfg rate id
	cfg_rate->dlength=6;	 	//����������Ϊ6���ֽ�.
	cfg_rate->measrate=measrate;//������,us
	cfg_rate->navrate=1;		//�������ʣ����ڣ����̶�Ϊ1
	cfg_rate->timeref=reftime; 	//�ο�ʱ��ΪGPSʱ��
	ublox_checksum((u8*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);
	//ublox_send_date((u8*)cfg_rate,sizeof(_ublox_cfg_rate));//�������ݸ�NEO-6M
	dev->write(uart_fops, (u8*)cfg_rate, sizeof(_ublox_cfg_rate));
	return ublox_cfg_ack_check();
}
//����һ�����ݸ�Ublox NEO-6M������ͨ������3����
//dbuf�����ݻ����׵�ַ
//len��Ҫ���͵��ֽ���
void ublox_send_date(u8* dbuf,u16 len)
{
	//u16 j;
	//for(j=0;j<len;j++)//ѭ����������
	//{
	//	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������
	//	USART3->DR=dbuf[j];
	//}
}
#endif



#include "gps.h"
#include <math.h>

#include "stdio.h"
#include "stdarg.h"
#include "string.h"

namespace driver {

gps::gps(PCSTR name, s32 id) :
    device(name, id)
{

}

gps::~gps(void)
{

}

s32 gps::probe(uart *puart)
{
    ASSERT(puart != NULL);
    _uart = puart;

    u8 buf[] = {
        "$GNRMC,111808.00,A,3106.06235,N,12115.08438,E,0.204,,310716,,,A*6C    \
        $GNVTG,,T,,M,0.204,N,0.378,K,A*37                                     \
        $GNGGA,111808.00,3106.06235,N,12115.08438,E,1,09,1.15,63.4,M,9.4,M,,*44   \
        $GNGSA,A,3,13,06,19,15,05,29,12,02,,,,,1.93,1.15,1.55*14                  \
        $GNGSA,A,3,83,,,,,,,,,,,,1.93,1.15,1.55*18                                \
        $GPGSV,4,1,15,02,71,050,23,05,61,314,30,06,34,104,32,07,12,072,*7B        \
        $GPGSV,4,2,15,09,02,038,,12,07,237,25,13,44,186,32,15,16,211,21*70        \
        $GPGSV,4,3,15,19,16,161,24,20,20,269,,25,05,271,,29,29,314,29*73          \
        $GPGSV,4,4,15,30,14,098,,42,46,140,,50,46,140,*49                         \
        $GLGSV,2,1,06,68,42,021,,70,15,239,26,82,14,151,24,83,70,150,28*6A        \
        $GLGSV,2,2,06,84,50,331,17,,,,24*5B                                       \
        $GNGLL,3106.06235,N,12115.08438,E,111808.00,A,A*71"
    };

    nmea_msg gpsx;
    memset(&gpsx, 0, sizeof(nmea_msg));
    gps::parse(&gpsx, buf);

    gps::init();

    return 0;

fail0:
    return -1;
}

s32 gps::init(void)
{
	_uart->open();
#if 0
	u8 key = 0xff;
	//���ö�λ��Ϣ�����ٶ�Ϊ1000ms,˳���ж�GPSģ���Ƿ���λ.
	if(ublox_cfg_rate(1000, 1) != 0)
	{
   		INF("NEO-7M setting....\n");
		while((ublox_cfg_rate(1000, 1) != 0) && key)	//�����ж�,ֱ�����Լ�鵽NEO-7M,�����ݱ���ɹ�
		{
			_uart->set_baudrate(uartbanud);			//��ʼ������3������Ϊ9600(EEPROMû�б������ݵ�ʱ��,������Ϊ9600.)
	  		ublox_cfg_prt(38400);			//��������ģ��Ĳ�����Ϊ38400
			ublox_cfg_yp(1000000, 100000, 1);	//����PPSΪ1�������1��,������Ϊ100ms
			key = Ublox_Cfg_Cfg_Save();		//��������
		}
		INF("NEO-7M set done.\n");
		mdelay(500);
	}
#endif
	return 0;
}


s32 gps::read_raw(void)
{
#if 0
#define GPS_SIZE 100
	u8 gps_string[GPS_SIZE] = {0};
	uart_this->read(uart_this, gps_string, GPS_SIZE);
	/* TODO:ͨ���ж���β�س�һ�������ݣ������������ֽ��� */
	u32 len = 50;

	gps_string[len]=0; 					//�Զ���ӽ�����
	parse(buf,(u8*)gps_string);	//�����ַ���
	//Gps_Msg_Show(); 					//��ʾ��Ϣ
	INF("\r\n%s\r\n",gps_string);		//���ͽ��յ������ݵ�����1
#endif
    return 0;
}


//��buf����õ���cx���������ڵ�λ��
//����ֵ:0~0XFE,����������λ�õ�ƫ��.
//       0XFF,�������ڵ�cx������
u8 gps::comma_pos(u8 *buf, u8 cx)
{
	u8 *p = buf;
	while (cx)
	{
		if(*buf == '*' || *buf < ' ' || *buf > 'z')
			return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf == ',')
			cx--;
		buf++;
	}
	return buf - p;
}

//m^n����
//����ֵ:m^n�η�.
u32 gps::NMEA_Pow(u8 m, u8 n)
{
	u32 result = 1;
	while(n--)
		result *= m;

	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int gps::str2num(u8 *buf, u8*dx)
{
	u8 *p = buf;
	u32 ires = 0, fres = 0;
	u8 ilen = 0, flen = 0, i;
	u8 mask = 0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p == '-') {				//�Ǹ���
			mask |= 0X02;
			p++;
		}
		if(*p == ',' || (*p=='*'))		//����������
			break;
		if(*p == '.') {				//����С������
			mask|=0X01;
			p++;
		}
		else if(*p > '9' || (*p < '0'))	//�зǷ��ַ�
		{
			ilen = 0;
			flen = 0;
			break;
		}
		if(mask & 0X01)
			flen++;
		else
			ilen++;
		p++;
	}
	if(mask & 0X02)			//ȥ������
		buf++;
	for(i=0; i < ilen; i++) {	//�õ�������������

		ires += NMEA_Pow(10, ilen - 1 - i) * (buf[i] - '0');
	}
	if(flen > 5)			//���ȡ5λС��
		flen = 5;
	*dx = flen;	 		//С����λ��
	for(i=0; i < flen; i++) {	//�õ�С����������

		fres += NMEA_Pow(10, flen-1-i) * (buf[ilen + 1 + i] - '0');
	}
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;
	return res;
}
//����GPGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void gps::GPGSV_parse(nmea_msg *gpsx, u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx = 0;
	u8 posx;
	p = buf;
	p1 = (u8*)strstr((const char *)p, "$GPGSV");
	len = p1[7] - '0';								//�õ�GPGSV������
	posx = comma_pos(p1, 3); 					//�õ��ɼ���������
	if(posx != 0XFF)gpsx->svnum = str2num(p1+posx, &dx);
	for(i=0; i < len; i++)
	{
		p1 = (u8*)strstr((const char *)p, "$GPGSV");
		for(j=0; j < 4; j++)
		{
			posx = comma_pos(p1,4+j*4);
			if(posx != 0XFF)
				gpsx->slmsg[slx].num = str2num(p1+posx, &dx);	//�õ����Ǳ��
			else
				break;
			posx = comma_pos(p1,5+j*4);
			if(posx != 0XFF)
				gpsx->slmsg[slx].eledeg = str2num(p1+posx, &dx);//�õ���������
			else
				break;
			posx = comma_pos(p1,6+j*4);
			if(posx != 0XFF)
				gpsx->slmsg[slx].azideg = str2num(p1+posx, &dx);//�õ����Ƿ�λ��
			else
				break;
			posx = comma_pos(p1, 7+j*4);
			if(posx != 0XFF)
				gpsx->slmsg[slx].sn = str2num(p1+posx, &dx);	//�õ����������
			else
				break;
			slx++;
		}
 		p=p1+1;//�л�����һ��GPGSV��Ϣ
	}
}
//����GPGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void gps::GPGGA_parse(nmea_msg *gpsx, u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	p1 = (u8*)strstr((const char *)buf, "$GPGGA");
	posx = comma_pos(p1,6);								//�õ�GPS״̬
	if(posx != 0XFF)
		gpsx->gpssta = str2num(p1+posx, &dx);
	posx = comma_pos(p1,7);								//�õ����ڶ�λ��������
	if(posx != 0XFF)
		gpsx->posslnum = str2num(p1+posx, &dx);
	posx = comma_pos(p1,9);								//�õ����θ߶�
	if(posx != 0XFF)
		gpsx->altitude = str2num(p1+posx, &dx);
}
//����GPGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void gps::GPGSA_parse(nmea_msg *gpsx, u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	u8 i;
	p1 = (u8*)strstr((const char *)buf, "$GPGSA");
	posx = comma_pos(p1, 2);								//�õ���λ����
	if(posx != 0XFF)
		gpsx->fixmode = str2num(p1+posx, &dx);
	for(i=0; i < 12; i++)										//�õ���λ���Ǳ��
	{
		posx=comma_pos(p1, 3+i);
		if(posx != 0XFF)
			gpsx->possl[i] = str2num(p1+posx, &dx);
		else
			break;
	}
	posx=comma_pos(p1, 15);								//�õ�PDOPλ�þ�������
	if(posx != 0XFF)
		gpsx->pdop = str2num(p1+posx, &dx);
	posx=comma_pos(p1, 16);								//�õ�HDOPλ�þ�������
	if(posx != 0XFF)
		gpsx->hdop = str2num(p1+posx, &dx);
	posx=comma_pos(p1, 17);								//�õ�VDOPλ�þ�������
	if(posx != 0XFF)
		gpsx->vdop = str2num(p1+posx, &dx);
}
//����GPRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void gps::GPRMC_parse(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	u32 temp;
	float rs;
	p1 = (u8*)strstr((const char *)buf, "GPRMC");//"$GPRMC",������&��GPRMC�ֿ������,��ֻ�ж�GPRMC.
	posx = comma_pos(p1, 1);								//�õ�UTCʱ��
	if(posx != 0XFF)
	{
		temp = str2num(p1+posx, &dx) / NMEA_Pow(10, dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour = temp / 10000;
		gpsx->utc.min = (temp / 100) % 100;
		gpsx->utc.sec = temp % 100;
	}
	posx = comma_pos(p1, 3);								//�õ�γ��
	if(posx != 0XFF)
	{
		temp = str2num(p1+posx, &dx);
		gpsx->latitude = temp / NMEA_Pow(10, dx+2);	//�õ���
		rs=temp % NMEA_Pow(10, dx+2);				//�õ�'
		gpsx->latitude = gpsx->latitude*NMEA_Pow(10, 5) + (rs*NMEA_Pow(10, 5-dx)) / 60;//ת��Ϊ��
	}
	posx = comma_pos(p1, 4);								//��γ���Ǳ�γ
	if(posx != 0XFF)
		gpsx->nshemi = *(p1+posx);
 	posx = comma_pos(p1, 5);								//�õ�����
	if(posx != 0XFF)
	{
		temp = str2num(p1+posx, &dx);
		gpsx->longitude = temp / NMEA_Pow(10, dx+2);	//�õ���
		rs= temp % NMEA_Pow(10, dx+2);				//�õ�'
		gpsx->longitude = gpsx->longitude * NMEA_Pow(10, 5) + (rs * NMEA_Pow(10, 5-dx)) / 60;//ת��Ϊ��
	}
	posx = comma_pos(p1, 6);								//������������
	if(posx != 0XFF)
		gpsx->ewhemi = *(p1 + posx);
	posx = comma_pos(p1, 9);								//�õ�UTC����
	if(posx != 0XFF)
	{
		temp = str2num(p1+posx, &dx);		 				//�õ�UTC����
		gpsx->utc.date = temp / 10000;
		gpsx->utc.month = (temp / 100) % 100;
		gpsx->utc.year = 2000 + temp % 100;
	}
}
//����GPVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void gps::GPVTG_parse(nmea_msg *gpsx, u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	p1 = (u8*)strstr((const char *)buf, "$GPVTG");
	posx = comma_pos(p1, 7);								//�õ���������
	if(posx != 0XFF)
	{
		gpsx->speed = str2num(p1+posx, &dx);
		if(dx<3)gpsx->speed *= NMEA_Pow(10, 3-dx);	 	 		//ȷ������1000��
	}
}
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void gps::parse(nmea_msg *gpsx, u8 *buf)
{
	GPGSV_parse(gpsx, buf);	//GPGSV����
	//GPGGA_parse(gpsx, buf);	//GPGGA����
	GPGSA_parse(gpsx, buf);	//GPGSA����
	GPRMC_parse(gpsx, buf);	//GPRMC����
	GPVTG_parse(gpsx, buf);	//GPVTG����
}

//GPSУ��ͼ���
//buf:���ݻ������׵�ַ
//len:���ݳ���
//cka,ckb:����У����.
void gps::ublox_checksum(u8 *buf, u16 len, u8* cka, u8*ckb)
{
	u16 i;
	*cka = 0;
	*ckb = 0;
	for(i=0; i<len; i++) {
		*cka = *cka + buf[i];
		*ckb = *ckb + *cka;
	}
}

}


/***********************************************************************
** End of file
***********************************************************************/


