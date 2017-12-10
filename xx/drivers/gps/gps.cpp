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
//UBLOX NEO-6M 配置(清除,保存,加载等)结构体
__packed typedef struct
{
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG CFG ID:0X0906 (小端模式)
	u16 dlength;				//数据长度 12/13
	u32 clearmask;				//子区域清除掩码(1有效)
	u32 savemask;				//子区域保存掩码
	u32 loadmask;				//子区域加载掩码
	u8  devicemask; 		  	//目标器件选择掩码	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	u8  cka;		 			//校验CK_A
	u8  ckb;			 		//校验CK_B
}_ublox_cfg_cfg;

//UBLOX NEO-6M 消息设置结构体
__packed typedef struct
{
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG MSG ID:0X0106 (小端模式)
	u16 dlength;				//数据长度 8
	u8  msgclass;				//消息类型(F0 代表NMEA消息格式)
	u8  msgid;					//消息 ID
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	u8  iicset;					//IIC消输出设置    0,关闭;1,使能.
	u8  uart1set;				//UART1输出设置	   0,关闭;1,使能.
	u8  uart2set;				//UART2输出设置	   0,关闭;1,使能.
	u8  usbset;					//USB输出设置	   0,关闭;1,使能.
	u8  spiset;					//SPI输出设置	   0,关闭;1,使能.
	u8  ncset;					//未知输出设置	   默认为1即可.
 	u8  cka;			 		//校验CK_A
	u8  ckb;			    	//校验CK_B
}_ublox_cfg_msg;

//UBLOX NEO-6M UART端口设置结构体
__packed typedef struct
{
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG PRT ID:0X0006 (小端模式)
	u16 dlength;				//数据长度 20
	u8  portid;					//端口号,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	u8  reserved;				//保留,设置为0
	u16 txready;				//TX Ready引脚设置,默认为0
	u32 mode;					//串口工作模式设置,奇偶校验,停止位,字节长度等的设置.
 	u32 baudrate;				//波特率设置
 	u16 inprotomask;		 	//输入协议激活屏蔽位  默认设置为0X07 0X00即可.
 	u16 outprotomask;		 	//输出协议激活屏蔽位  默认设置为0X07 0X00即可.
 	u16 reserved4; 				//保留,设置为0
 	u16 reserved5; 				//保留,设置为0
 	u8  cka;			 		//校验CK_A
	u8  ckb;			    	//校验CK_B
}_ublox_cfg_prt;

//UBLOX NEO-6M 时钟脉冲配置结构体
__packed typedef struct
{
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG TP ID:0X0706 (小端模式)
	u16 dlength;				//数据长度
	u32 interval;				//时钟脉冲间隔,单位为us
	u32 length;				 	//脉冲宽度,单位为us
	signed char status;			//时钟脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.
	u8 timeref;			   		//参考时间:0,UTC时间;1,GPS时间;2,当地时间.
	u8 flags;					//时间脉冲设置标志
	u8 reserved;				//保留
 	signed short antdelay;	 	//天线延时
 	signed short rfdelay;		//RF延时
	signed int userdelay; 	 	//用户延时
	u8 cka;						//校验CK_A
	u8 ckb;						//校验CK_B
}_ublox_cfg_tp;

//UBLOX NEO-6M 刷新速率配置结构体
__packed typedef struct
{
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG RATE ID:0X0806 (小端模式)
	u16 dlength;				//数据长度
	u16 measrate;				//测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
	u16 navrate;				//导航速率（周期），固定为1
	u16 timeref;				//参考时间：0=UTC Time；1=GPS Time；
 	u8  cka;					//校验CK_A
	u8  ckb;					//校验CK_B
}_ublox_cfg_rate;


/////////////////////////////////////////UBLOX 配置代码/////////////////////////////////////
//检查CFG配置执行情况
//返回值:0,ACK成功
//       1,接收超时错误
//       2,没有找到同步字符
//       3,接收到NACK应答
u8 ublox_cfg_ack_check(void)
{
	u16 len=0,i;
	u8 rval=0;
	while((USART3_RX_STA & 0X8000) == 0 && len < 100)//等待接收到应答
	{
		len++;
		delay_ms(5);
	}
	if(len < 250)   	//超时错误.
	{
		len = USART3_RX_STA & 0X7FFF;	//此次接收到的数据长度
		for(i=0; i < len; i++)
			if(USART3_RX_BUF[i]==0XB5)
				break;//查找同步字符 0XB5
		if(i == len)
			rval = 2;						//没有找到同步字符
		else if(USART3_RX_BUF[i+3] == 0X00)
			rval = 3;//接收到NACK应答
		else
			rval = 0;	   						//接收到ACK应答
	}
	else
	rval = 1;								//接收超时错误
	USART3_RX_ST A= 0;							//清除接收
	return rval;
}
//配置保存
//将当前配置保存在外部EEPROM里面
//返回值:0,执行成功;1,执行失败.
u8 Ublox_Cfg_Cfg_Save(void)
{
	u8 i;
	_ublox_cfg_cfg cfg;
	_ublox_cfg_cfg *cfg_cfg=&cfg;
	cfg_cfg->header=0X62B5;		//cfg header
	cfg_cfg->id=0X0906;			//cfg cfg id
	cfg_cfg->dlength=13;		//数据区长度为13个字节.
	cfg_cfg->clearmask=0;		//清除掩码为0
	cfg_cfg->savemask=0XFFFF; 	//保存掩码为0XFFFF
	cfg_cfg->loadmask=0; 		//加载掩码为0
	cfg_cfg->devicemask=4; 		//保存在EEPROM里面
	ublox_checksum((u8*)(&cfg_cfg->id),sizeof(_ublox_cfg_cfg)-4,&cfg_cfg->cka,&cfg_cfg->ckb);
	ublox_send_date((u8*)cfg_cfg,sizeof(_ublox_cfg_cfg));//发送数据给NEO-6M
	for(i=0;i<6;i++)if(ublox_cfg_ack_check()==0)break;		//EEPROM写入需要比较久时间,所以连续判断多次
	return i==6?1:0;
}
//配置NMEA输出信息格式
//msgid:要操作的NMEA消息条目,具体见下面的参数表
//      00,GPGGA;01,GPGLL;02,GPGSA;
//		03,GPGSV;04,GPRMC;05,GPVTG;
//		06,GPGRS;07,GPGST;08,GPZDA;
//		09,GPGBS;0A,GPDTM;0D,GPGNS;
//uart1set:0,输出关闭;1,输出开启.
//返回值:0,执行成功;其他,执行失败.
u8 Ublox_Cfg_Msg(u8 msgid,u8 uart1set)
{
	_ublox_cfg_msg msg;
	_ublox_cfg_msg *cfg_msg=&msg;
	cfg_msg->header=0X62B5;		//cfg header
	cfg_msg->id=0X0106;			//cfg msg id
	cfg_msg->dlength=8;			//数据区长度为8个字节.
	cfg_msg->msgclass=0XF0;  	//NMEA消息
	cfg_msg->msgid=msgid; 		//要操作的NMEA消息条目
	cfg_msg->iicset=1; 			//默认开启
	cfg_msg->uart1set=uart1set; //开关设置
	cfg_msg->uart2set=1; 	 	//默认开启
	cfg_msg->usbset=1; 			//默认开启
	cfg_msg->spiset=1; 			//默认开启
	cfg_msg->ncset=1; 			//默认开启
	ublox_checksum((u8*)(&cfg_msg->id),sizeof(_ublox_cfg_msg)-4,&cfg_msg->cka,&cfg_msg->ckb);
	ublox_send_date((u8*)cfg_msg,sizeof(_ublox_cfg_msg));//发送数据给NEO-6M
	return ublox_cfg_ack_check();
}
//配置NMEA输出信息格式
//baudrate:波特率,4800/9600/19200/38400/57600/115200/230400
//返回值:0,执行成功;其他,执行失败(这里不会返回0了)
u8 ublox_cfg_prt(u32 baudrate)
{
	_ublox_cfg_prt cfg;
	_ublox_cfg_prt *cfg_prt=&cfg;
	cfg_prt->header=0X62B5;		//cfg header
	cfg_prt->id=0X0006;			//cfg prt id
	cfg_prt->dlength=20;		//数据区长度为20个字节.
	cfg_prt->portid=1;			//操作串口1
	cfg_prt->reserved=0;	 	//保留字节,设置为0
	cfg_prt->txready=0;	 		//TX Ready设置为0
	cfg_prt->mode=0X08D0; 		//8位,1个停止位,无校验位
	cfg_prt->baudrate=baudrate; //波特率设置
	cfg_prt->inprotomask=0X0007;//0+1+2
	cfg_prt->outprotomask=0X0007;//0+1+2
 	cfg_prt->reserved4=0; 		//保留字节,设置为0
 	cfg_prt->reserved5=0; 		//保留字节,设置为0
	ublox_checksum((u8*)(&cfg_prt->id),sizeof(_ublox_cfg_prt)-4,&cfg_prt->cka,&cfg_prt->ckb);
	ublox_send_date((u8*)cfg_prt,sizeof(_ublox_cfg_prt));//发送数据给NEO-6M
	delay_ms(200);				//等待发送完成
	usart3_init(baudrate);	//重新初始化串口3
	return ublox_cfg_ack_check();//这里不会反回0,因为UBLOX发回来的应答在串口重新初始化的时候已经被丢弃了.
}
//配置UBLOX NEO-6的时钟脉冲输出
//interval:脉冲间隔(us)
//length:脉冲宽度(us)
//status:脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.
//返回值:0,发送成功;其他,发送失败.
u8 ublox_cfg_yp(u32 interval,u32 length,signed char status)
{
	_ublox_cfg_tp tp;
	_ublox_cfg_tp *cfg_tp=&tp;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//数据区长度为20个字节.
	cfg_tp->interval=interval;	//脉冲间隔,us
	cfg_tp->length=length;		//脉冲宽度,us
	cfg_tp->status=status;	   	//时钟脉冲配置
	cfg_tp->timeref=0;			//参考UTC 时间
	cfg_tp->flags=0;			//flags为0
	cfg_tp->reserved=0;		 	//保留位为0
	cfg_tp->antdelay=820;    	//天线延时为820ns
	cfg_tp->rfdelay=0;    		//RF延时为0ns
	cfg_tp->userdelay=0;    	//用户延时为0ns
	ublox_checksum((u8*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	ublox_send_date((u8*)cfg_tp,sizeof(_ublox_cfg_tp));//发送数据给NEO-6M
	return ublox_cfg_ack_check();
}
//配置UBLOX NEO-6的更新速率
//measrate:测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
//reftime:参考时间，0=UTC Time；1=GPS Time（一般设置为1）
//返回值:0,发送成功;其他,发送失败.
u8 ublox_cfg_rate(u16 measrate,u8 reftime)
{
	_ublox_cfg_rate rate;
	_ublox_cfg_rate *cfg_rate=&rate;
 	if(measrate<200)return 1;	//小于200ms，直接退出
 	cfg_rate->header=0X62B5;	//cfg header
	cfg_rate->id=0X0806;	 	//cfg rate id
	cfg_rate->dlength=6;	 	//数据区长度为6个字节.
	cfg_rate->measrate=measrate;//脉冲间隔,us
	cfg_rate->navrate=1;		//导航速率（周期），固定为1
	cfg_rate->timeref=reftime; 	//参考时间为GPS时间
	ublox_checksum((u8*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);
	//ublox_send_date((u8*)cfg_rate,sizeof(_ublox_cfg_rate));//发送数据给NEO-6M
	dev->write(uart_fops, (u8*)cfg_rate, sizeof(_ublox_cfg_rate));
	return ublox_cfg_ack_check();
}
//发送一批数据给Ublox NEO-6M，这里通过串口3发送
//dbuf：数据缓存首地址
//len：要发送的字节数
void ublox_send_date(u8* dbuf,u16 len)
{
	//u16 j;
	//for(j=0;j<len;j++)//循环发送数据
	//{
	//	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕
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
	//设置定位信息更新速度为1000ms,顺便判断GPS模块是否在位.
	if(ublox_cfg_rate(1000, 1) != 0)
	{
   		INF("NEO-7M setting....\n");
		while((ublox_cfg_rate(1000, 1) != 0) && key)	//持续判断,直到可以检查到NEO-7M,且数据保存成功
		{
			_uart->set_baudrate(uartbanud);			//初始化串口3波特率为9600(EEPROM没有保存数据的时候,波特率为9600.)
	  		ublox_cfg_prt(38400);			//重新设置模块的波特率为38400
			ublox_cfg_yp(1000000, 100000, 1);	//设置PPS为1秒钟输出1次,脉冲宽度为100ms
			key = Ublox_Cfg_Cfg_Save();		//保存配置
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
	/* TODO:通过判断首尾截出一整包数据，并计算数据字节数 */
	u32 len = 50;

	gps_string[len]=0; 					//自动添加结束符
	parse(buf,(u8*)gps_string);	//分析字符串
	//Gps_Msg_Show(); 					//显示信息
	INF("\r\n%s\r\n",gps_string);		//发送接收到的数据到串口1
#endif
    return 0;
}


//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号
u8 gps::comma_pos(u8 *buf, u8 cx)
{
	u8 *p = buf;
	while (cx)
	{
		if(*buf == '*' || *buf < ' ' || *buf > 'z')
			return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf == ',')
			cx--;
		buf++;
	}
	return buf - p;
}

//m^n函数
//返回值:m^n次方.
u32 gps::NMEA_Pow(u8 m, u8 n)
{
	u32 result = 1;
	while(n--)
		result *= m;

	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int gps::str2num(u8 *buf, u8*dx)
{
	u8 *p = buf;
	u32 ires = 0, fres = 0;
	u8 ilen = 0, flen = 0, i;
	u8 mask = 0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p == '-') {				//是负数
			mask |= 0X02;
			p++;
		}
		if(*p == ',' || (*p=='*'))		//遇到结束了
			break;
		if(*p == '.') {				//遇到小数点了
			mask|=0X01;
			p++;
		}
		else if(*p > '9' || (*p < '0'))	//有非法字符
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
	if(mask & 0X02)			//去掉负号
		buf++;
	for(i=0; i < ilen; i++) {	//得到整数部分数据

		ires += NMEA_Pow(10, ilen - 1 - i) * (buf[i] - '0');
	}
	if(flen > 5)			//最多取5位小数
		flen = 5;
	*dx = flen;	 		//小数点位数
	for(i=0; i < flen; i++) {	//得到小数部分数据

		fres += NMEA_Pow(10, flen-1-i) * (buf[ilen + 1 + i] - '0');
	}
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;
	return res;
}
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void gps::GPGSV_parse(nmea_msg *gpsx, u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx = 0;
	u8 posx;
	p = buf;
	p1 = (u8*)strstr((const char *)p, "$GPGSV");
	len = p1[7] - '0';								//得到GPGSV的条数
	posx = comma_pos(p1, 3); 					//得到可见卫星总数
	if(posx != 0XFF)gpsx->svnum = str2num(p1+posx, &dx);
	for(i=0; i < len; i++)
	{
		p1 = (u8*)strstr((const char *)p, "$GPGSV");
		for(j=0; j < 4; j++)
		{
			posx = comma_pos(p1,4+j*4);
			if(posx != 0XFF)
				gpsx->slmsg[slx].num = str2num(p1+posx, &dx);	//得到卫星编号
			else
				break;
			posx = comma_pos(p1,5+j*4);
			if(posx != 0XFF)
				gpsx->slmsg[slx].eledeg = str2num(p1+posx, &dx);//得到卫星仰角
			else
				break;
			posx = comma_pos(p1,6+j*4);
			if(posx != 0XFF)
				gpsx->slmsg[slx].azideg = str2num(p1+posx, &dx);//得到卫星方位角
			else
				break;
			posx = comma_pos(p1, 7+j*4);
			if(posx != 0XFF)
				gpsx->slmsg[slx].sn = str2num(p1+posx, &dx);	//得到卫星信噪比
			else
				break;
			slx++;
		}
 		p=p1+1;//切换到下一个GPGSV信息
	}
}
//分析GPGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void gps::GPGGA_parse(nmea_msg *gpsx, u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	p1 = (u8*)strstr((const char *)buf, "$GPGGA");
	posx = comma_pos(p1,6);								//得到GPS状态
	if(posx != 0XFF)
		gpsx->gpssta = str2num(p1+posx, &dx);
	posx = comma_pos(p1,7);								//得到用于定位的卫星数
	if(posx != 0XFF)
		gpsx->posslnum = str2num(p1+posx, &dx);
	posx = comma_pos(p1,9);								//得到海拔高度
	if(posx != 0XFF)
		gpsx->altitude = str2num(p1+posx, &dx);
}
//分析GPGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void gps::GPGSA_parse(nmea_msg *gpsx, u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	u8 i;
	p1 = (u8*)strstr((const char *)buf, "$GPGSA");
	posx = comma_pos(p1, 2);								//得到定位类型
	if(posx != 0XFF)
		gpsx->fixmode = str2num(p1+posx, &dx);
	for(i=0; i < 12; i++)										//得到定位卫星编号
	{
		posx=comma_pos(p1, 3+i);
		if(posx != 0XFF)
			gpsx->possl[i] = str2num(p1+posx, &dx);
		else
			break;
	}
	posx=comma_pos(p1, 15);								//得到PDOP位置精度因子
	if(posx != 0XFF)
		gpsx->pdop = str2num(p1+posx, &dx);
	posx=comma_pos(p1, 16);								//得到HDOP位置精度因子
	if(posx != 0XFF)
		gpsx->hdop = str2num(p1+posx, &dx);
	posx=comma_pos(p1, 17);								//得到VDOP位置精度因子
	if(posx != 0XFF)
		gpsx->vdop = str2num(p1+posx, &dx);
}
//分析GPRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void gps::GPRMC_parse(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	u32 temp;
	float rs;
	p1 = (u8*)strstr((const char *)buf, "GPRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	posx = comma_pos(p1, 1);								//得到UTC时间
	if(posx != 0XFF)
	{
		temp = str2num(p1+posx, &dx) / NMEA_Pow(10, dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour = temp / 10000;
		gpsx->utc.min = (temp / 100) % 100;
		gpsx->utc.sec = temp % 100;
	}
	posx = comma_pos(p1, 3);								//得到纬度
	if(posx != 0XFF)
	{
		temp = str2num(p1+posx, &dx);
		gpsx->latitude = temp / NMEA_Pow(10, dx+2);	//得到°
		rs=temp % NMEA_Pow(10, dx+2);				//得到'
		gpsx->latitude = gpsx->latitude*NMEA_Pow(10, 5) + (rs*NMEA_Pow(10, 5-dx)) / 60;//转换为°
	}
	posx = comma_pos(p1, 4);								//南纬还是北纬
	if(posx != 0XFF)
		gpsx->nshemi = *(p1+posx);
 	posx = comma_pos(p1, 5);								//得到经度
	if(posx != 0XFF)
	{
		temp = str2num(p1+posx, &dx);
		gpsx->longitude = temp / NMEA_Pow(10, dx+2);	//得到°
		rs= temp % NMEA_Pow(10, dx+2);				//得到'
		gpsx->longitude = gpsx->longitude * NMEA_Pow(10, 5) + (rs * NMEA_Pow(10, 5-dx)) / 60;//转换为°
	}
	posx = comma_pos(p1, 6);								//东经还是西经
	if(posx != 0XFF)
		gpsx->ewhemi = *(p1 + posx);
	posx = comma_pos(p1, 9);								//得到UTC日期
	if(posx != 0XFF)
	{
		temp = str2num(p1+posx, &dx);		 				//得到UTC日期
		gpsx->utc.date = temp / 10000;
		gpsx->utc.month = (temp / 100) % 100;
		gpsx->utc.year = 2000 + temp % 100;
	}
}
//分析GPVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void gps::GPVTG_parse(nmea_msg *gpsx, u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	p1 = (u8*)strstr((const char *)buf, "$GPVTG");
	posx = comma_pos(p1, 7);								//得到地面速率
	if(posx != 0XFF)
	{
		gpsx->speed = str2num(p1+posx, &dx);
		if(dx<3)gpsx->speed *= NMEA_Pow(10, 3-dx);	 	 		//确保扩大1000倍
	}
}
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void gps::parse(nmea_msg *gpsx, u8 *buf)
{
	GPGSV_parse(gpsx, buf);	//GPGSV解析
	//GPGGA_parse(gpsx, buf);	//GPGGA解析
	GPGSA_parse(gpsx, buf);	//GPGSA解析
	GPRMC_parse(gpsx, buf);	//GPRMC解析
	GPVTG_parse(gpsx, buf);	//GPVTG解析
}

//GPS校验和计算
//buf:数据缓存区首地址
//len:数据长度
//cka,ckb:两个校验结果.
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


