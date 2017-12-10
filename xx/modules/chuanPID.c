#include "CONTROL.h"
#include "IMU1.h"
#include "moto.h"
#include "RFdate.h"
#include <math.h>
extern T_RC_Data                         rc_d;                //Ò£¿ØÍ¨µÀÊý¾Ý;

extern u8 txbuf[4];         //·¢ËÍ»º³å
extern u8 rxbuf[4];         //½ÓÊÕ»º³å
extern u16 test1[3]; //½ÓÊÕµ½NRf24L01Êý¾Ý
extern S_INT16_XYZ ACC_F,GYRO_F;

PID PID_ROL,PID_PIT,PID_YAW;

extern S_INT16_XYZ        MPU6050_ACC_LAST,MPU6050_GYRO_LAST;


int Motor_Ele=0;                                           //¸©ÑöÆÚÍû
int Motor_Ail=0;                                           //ºá¹öÆÚÍû

//u8 ARMED = 0;

//float rol_i=0,pit_i=0,yaw_p=0;
float thr=0;

S_FLOAT_XYZ EXP_ANGLE ,DIF_ANGLE;
PID1 PID_Motor;
/*********************************/
float pitch_i,roll_i,Yaw_i;                         //»ý·ÖÏî
float pitch_old,roll_old,yaw_old;                   //½Ç¶È±£´æ
float pitch_d,roll_d,yaw_d;                         //Î¢·ÖÏî
float rc_pitch,rc_roll,rc_yaw;                      //×ËÌ¬½Ç

//Íâ»·PID²ÎÊý
float pitch_shell_kp=280;//30 140
float pitch_shell_kd=0;//
float pitch_shell_ki=0;//
float roll_shell_kp=250;//30
float roll_shell_kd=0;//10
float roll_shell_ki=0;//0.08
float yaw_shell_kp=1.5;//10;//30
float yaw_shell_kd=0;//10
float yaw_shell_ki=0;//0.08;//0.08
float pitch_shell_out,roll_shell_out,yaw_shell_out; //Íâ»·×ÜÊä³ö

//ÄÚ»·PID²ÎÊý
float pitch_core_kp=0.040;
float Pitch_core_kd=0.002;////0.007;//0.07;0.008;
float Roll_core_kp=0.040;//;
float Roll_core_kd=0.002;////0.007;//06;//0.07;
float Yaw_core_kp=0.046;//;
float Yaw_core_kd=0.012;////0.007;//06;//0.07;
float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//ÍÓÂÝÒÇ±£´æ
float pitch_core_kp_out,pitch_core_kd_out,Roll_core_kp_out,Roll_core_kd_out,yaw_core_kp_out,Yaw_core_kd_out;//ÄÚ»·µ¥ÏîÊä³ö
float pitch_core_out,Roll_core_out,Yaw_core_out;//ÄÚ»·×ÜÊä³ö

int16_t moto1=0,moto2=0,moto3=0,moto4=0;

float tempjd=0;
void CONTROL(float rol, float pit, float yaw)
{
    ////////////////////////Íâ»·½Ç¶È»·(PID)///////////////////////////////
    rc_pitch = (rc_d.pitch - 1500) / 20;
    pitch_i += (q_angle.pitch - rc_pitch);
    //-------------Pitch»ý·ÖÏÞ·ù----------------//
    if(pitch_i > 300) pitch_i = 300;
    else if(pitch_i < -300) pitch_i = -300;
    //-------------PitchÎ¢·Ö--------------------//
    pitch_d = q_angle.pitch - pitch_old;
    //-------------pitch  PID-------------------//
    pitch_shell_out = pitch_shell_kp*(q_angle.pitch - rc_pitch) + pitch_shell_ki*pitch_i + pitch_shell_kd*pitch_d;
    //½Ç¶È±£´æ
    pitch_old = q_angle.pitch;

    rc_roll = (rc_d.roll - 1500) / 20;
    roll_i += (q_angle.roll - rc_roll);
    //-------------Roll»ý·ÖÏÞ·ù----------------//
    if(roll_i > 300) roll_i = 300;
    else if(roll_i < -300) roll_i = -300;
    //-------------RollÎ¢·Ö--------------------//
    roll_d = q_angle.roll - roll_old;
    //-------------Roll  PID-------------------//
    roll_shell_out = roll_shell_kp*(q_angle.roll - rc_roll) + roll_shell_ki*roll_i + roll_shell_kd*roll_d;
    //------------Roll½Ç¶È±£´æ------------------//
    roll_old = q_angle.roll;


    rc_yaw=(rc_d.yaw-1500)*10;
    //-------------YawÎ¢·Ö--------------------//
    yaw_d=MPU6050_GYRO_LAST.Z - yaw_old;
    //-------------Roll  PID-------------------//
    yaw_shell_out  = yaw_shell_kp*(MPU6050_GYRO_LAST.Z-rc_yaw) + yaw_shell_ki*Yaw_i + yaw_shell_kd*yaw_d;
    //------------Roll½Ç¶È±£´æ------------------//
    yaw_old=MPU6050_GYRO_LAST.Z;

    ////////////////////////ÄÚ»·½ÇËÙ¶È»·(PD)///////////////////////////////
    pitch_core_kp_out = pitch_core_kp * (pitch_shell_out + MPU6050_GYRO_LAST.Y * 3.5);
    pitch_core_kd_out = Pitch_core_kd * (MPU6050_GYRO_LAST.Y   - Gyro_radian_old_y);
    Roll_core_kp_out  = Roll_core_kp  * (roll_shell_out  + MPU6050_GYRO_LAST.X *3.5);
    Roll_core_kd_out  = Roll_core_kd  * (MPU6050_GYRO_LAST.X   - Gyro_radian_old_x);
    yaw_core_kp_out  = Yaw_core_kp  * (yaw_shell_out  + MPU6050_GYRO_LAST.Z * 1);
    Yaw_core_kd_out  = Yaw_core_kd  * (MPU6050_GYRO_LAST.Z   - Gyro_radian_old_z);

    pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
    Roll_core_out  = Roll_core_kp_out  + Roll_core_kd_out;
    Yaw_core_out   = yaw_core_kp_out   + Yaw_core_kd_out;

    Gyro_radian_old_y = MPU6050_GYRO_LAST.X;
    Gyro_radian_old_x = MPU6050_GYRO_LAST.Y;
    Gyro_radian_old_z = MPU6050_GYRO_LAST.Z;   //´¢´æÀúÊ·Öµ

//--------------------½«Êä³öÖµÈÚºÏµ½ËÄ¸öµç»ú--------------------------------//


        if(rc_d.THROTTLE>1020)
        {
  thr=rc_d.THROTTLE- 1000;

//                if(rc_d.THROTTLE<=2000)
//                {
//  moto1=(int16_t)(thr  - pitch_core_out);//- yaw);
//        moto2=(int16_t)(thr  - pitch_core_out);//+ yaw);
//        moto3=(int16_t)(thr  + pitch_core_out);// - yaw);
//        moto4=(int16_t)(thr  + pitch_core_out);//+ yaw);

//  moto1=(int16_t)(thr  - Roll_core_out);//- yaw);
//        moto2=(int16_t)(thr  + Roll_core_out);//+ yaw);
//        moto3=(int16_t)(thr  + Roll_core_out);// - yaw);
//        moto4=(int16_t)(thr  - Roll_core_out);//+ yaw);

//  moto1=(int16_t)(thr  - Yaw_core_out);//- yaw);
//        moto2=(int16_t)(thr  + Yaw_core_out);//+ yaw);
//        moto3=(int16_t)(thr  - Yaw_core_out);// - yaw);
//        moto4=(int16_t)(thr  + Yaw_core_out);//+ yaw);

//moto1=(int16_t)(thr - Roll_core_out - pitch_core_out);
//moto2=(int16_t)(thr + Roll_core_out - pitch_core_out);
//moto3=(int16_t)(thr + Roll_core_out + pitch_core_out);
//moto4=(int16_t)(thr - Roll_core_out + pitch_core_out);
//
  moto1=(int16_t)(thr - Roll_core_out - pitch_core_out- Yaw_core_out);
        moto2=(int16_t)(thr + Roll_core_out - pitch_core_out+ Yaw_core_out);
        moto3=(int16_t)(thr + Roll_core_out + pitch_core_out- Yaw_core_out);
        moto4=(int16_t)(thr - Roll_core_out + pitch_core_out+ Yaw_core_out);

//                }
  }
        else
        {
                moto1 = 0;
                moto2 = 0;
                moto3 = 0;
                moto4 = 0;
        }
        MOTO_PWMRFLASH(moto1,moto2,moto3,moto4);//        Moto_PwmRflash(moto1,moto2,moto3,moto4);
}











/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
* ÎÄ¼þÃû £ºANO_FlyControl.cpp
* ÃèÊö £º·ÉÐÐ¿ØÖÆ
**********************************************************************************/
include "ANO_FlyControl.h"ANO_FlyControl fc;

/*
ÏÈÕû¶¨ÄÚ»·£¬ºóÕû¶¨Íâ»·¡£
²ÎÊýÕû¶¨ÕÒ×î¼Ñ£¬´ÓÐ¡µ½´óË³Ðò²é
ÏÈÊÇ±ÈÀýºó»ý·Ö£¬×îºóÔÙ°ÑÎ¢·Ö¼Ó
ÇúÏßÕñµ´ºÜÆµ·±£¬±ÈÀý¶ÈÅÌÒª·Å´ó
ÇúÏßÆ¯¸¡ÈÆ´óÍå£¬±ÈÀý¶ÈÅÌÍùÐ¡°â
ÇúÏßÆ«Àë»Ø¸´Âý£¬»ý·ÖÊ±¼äÍùÏÂ½µ
ÇúÏß²¨¶¯ÖÜÆÚ³¤£¬»ý·ÖÊ±¼äÔÙ¼Ó³¤
ÇúÏßÕñµ´ÆµÂÊ¿ì£¬ÏÈ°ÑÎ¢·Ö½µÏÂÀ´
¶¯²î´óÀ´²¨¶¯Âý¡£Î¢·ÖÊ±¼äÓ¦¼Ó³¤
ÀíÏëÇúÏßÁ½¸ö²¨£¬Ç°¸ßºóµÍ4±È1
*/

/*
ROLLºÍPITÖáÏò°´ÕÕÒÔÉÏ¹«Ê½¼ÆËãPIDÊä³ö£¬µ«YAWÖá±È½ÏÌØÊâ£¬ÒòÎªÆ«º½½Ç·¨Ïß·½Ïò¸ÕºÃºÍµØÇòÖØÁ¦Æ½ÐÐ£¬
Õâ¸ö·½ÏòµÄ½Ç¶ÈÎÞ·¨ÓÉ¼ÓËÙ¶È¼ÆÖ±½Ó²âµÃ£¬ÐèÒªÔö¼ÓÒ»¸öµç×ÓÂÞÅÌÀ´Ìæ´ú¼ÓËÙ¶È¼Æ¡£Èç¹û²»Ê¹ÓÃÂÞÅÌµÄ»°£¬
ÎÒÃÇ¿ÉÒÔµ¥´¿µÄÍ¨¹ý½ÇËÙ¶È»ý·ÖÀ´²âµÃÆ«º½½Ç£¬È±µãÊÇÓÉÓÚ»ý·Ö»·½ÚÖÐ´æÔÚ»ý·ÖÆ¯ÒÆ£¬Æ«º½½ÇËæ×ÅÊ±¼äµÄÍÆÒÆ
»áÆ«²îÔ½À´Ô½´ó¡£ÎÒÃÇ²»Ê¹ÓÃÂÞÅÌ¾ÍÃ»ÓÐ±ÈÀýÏî£¬Ö»½öÊ¹ÓÃÎ¢·Ö»·½ÚÀ´¿ØÖÆ¡£
*/

ANO_FlyControl::ANO_FlyControl()
{
    yawRate = 120;
    //ÖØÖÃPID²ÎÊý
    PID_Reset();
}

//ÖØÖÃPID²ÎÊý
void ANO_FlyControl:ID_Reset(void)
{
    //ÒòÎªYAW½Ç¶È»áÆ¯ÒÆ£¬ËùÒÔ²ÎÊýºÍROLL¡¢PITCH²»Ò»Ñù
    pid[PIDROLL].set_pid(70, 15, 120, 2000000); //ROLL½Ç¶ÈµÄÄÚ»·¿ØÖÆÏµÊý,20000:»ý·ÖÉÏÏÞ
    pid[PIDPITCH].set_pid(70, 30, 120, 2000000);//PITCH½Ç¶ÈµÄÄÚ»·¿ØÖÆÏµÊý
    pid[PIDYAW].set_pid(100, 50, 0, 2000000); //YAW½Ç¶ÈµÄÄÚ»·¿ØÖÆÏµÊý

    pid[PIDLEVEL].set_pid(280, 0, 0, 0); //Íâ»·¿ØÖÆÏµÊý
    pid[PIDMAG].set_pid(15, 0, 0, 0); //µç×ÓÂÞÅÌ¿ØÖÆÏµÊý
}

/*
¡¾É¨Ã¤ÖªÊ¶¡¿
´®¼¶PID£º²ÉÓÃµÄ½Ç¶ÈPºÍ½ÇËÙ¶ÈPIDµÄË«±Õ»·PIDËã·¨------>½Ç¶ÈµÄÎó²î±»×÷ÎªÆÚÍûÊäÈëµ½½ÇËÙ¶È¿ØÖÆÆ÷ÖÐ £¨½Ç¶ÈµÄÎ¢·Ö¾ÍÊÇ½ÇËÙ¶È£©
¶ÔÓÚ±¾ÏµÍ³Ôò²ÉÓÃÁË½«½Ç¶È¿ØÖÆÓë½ÇËÙ¶È¿ØÖÆ¼¶ÁªµÄ·½Ê½×é³ÉÕû¸ö´®¼¶ PID ¿ØÖÆÆ÷¡£

´®¼¶ PID Ëã·¨ÖÐ£¬½ÇËÙ¶ÈÄÚ»·Õ¼×Å¼«ÎªÖØÒªµÄµØÎ»¡£ÔÚ¶ÔËÄÐýÒí·ÉÐÐµÄÎïÀíÄ£ÐÍ½ø
ÐÐ·ÖÎöºó£¬¿ÉÒÔÖªµÀÔì³ÉÏµÍ³²»ÎÈ¶¨µÄÎïÀí±íÏÖÖ®Ò»¾ÍÊÇ²»ÎÈ¶¨µÄ½ÇËÙ¶È¡£
Òò´Ë£¬ÈôÄÜ¹»Ö±½Ó¶ÔÏµÍ³µÄ½ÇËÙ¶È½øÐÐ½ÏºÃµÄ±Õ»·¿ØÖÆ£¬±ØÈ»»á¸ÄÉÆÏµÍ³µÄ¶¯Ì¬ÌØÐÔ
¼°ÆäÎÈ¶¨ÐÔ£¬Í¨³£Ò²°Ñ½ÇËÙ¶ÈÄÚ»·³ÆÎªÔöÎÈ»·½Ú¡£¶ø½Ç¶ÈÍâ»·µÄ×÷ÓÃÔòÌåÏÖÔÚ¶ÔËÄÐýÒí·É
ÐÐÆ÷µÄ×ËÌ¬½ÇµÄ¾«È·¿ØÖÆ¡£
Íâ»·£ºÊäÈëÎª½Ç¶È,Êä³öÎª½ÇËÙ¶È
ÄÚ»·£ºÊäÈëÎª½ÇËÙ¶È£¬Êä³öÎªPWMÔöÁ¿
Ê¹ÓÃ´®¼¶pid£¬·ÖÎª£º½Ç¶È»·¿ØÖÆpid»·£¬ºÍ½ÇËÙ¶È¿ØÖÆ»·ÎÈ¶¨»·¡£Ö÷µ÷Îª½Ç¶È»·£¨Íâ»·£©£¬¸±µ÷Îª½ÇËÙ¶È»·£¨ÄÚ»·£©¡£
²ÎÊýÕû¶¨Ô­ÔòÎªÏÈÄÚºóÍâ£¬¹ÊÔÚÕû¶¨ÄÚ»·Ê±½«Íâ»·µÄPID¾ùÉèÎª0
ËùÎ½Íâ»·¾ÍÊÇÖ»ÊÇÒ»¸öPÔÚÆð×÷ÓÃ£¬Ò²¾ÍÊÇ±ÈÀýÔÚÆð×÷ÓÃ£»PÒ²¾ÍÊÇÐÞÕýÁ¦¶È£¬Ô½´óÔ½ÈÝÒ×Ê¹·É»úÕðµ´¡£
Õðµ´µÄÌØµãÊÇ£ºÆµÂÊÐ¡¡¢·ù¶È´ó
*/

/*
¡¾ºá¹ö£¨Roll£©ºÍ¸©Ñö£¨Pitch£©µÄ¿ØÖÆËã·¨¡¿
ºá¹ö£¨Roll£©ºÍ¸©Ñö£¨Pitch£©µÄ¿ØÖÆËã·¨ÊÇÒ»ÑùµÄ£¬¿ØÖÆ²ÎÊýÒ²±È½Ï½Ó½ü¡£

Ê×ÏÈµÃµ½Öá×ËÌ¬µÄ½Ç¶È²î£¨angle error£©£¬½«Õâ¸öÖµ³ËÒÔ½Ç¶ÈÏµÊýp
ºóÏÞ·ù£¨ÏÞ·ù±ØÐëÓÐ£¬·ñÔò¾çÁÒ´ò¶æÊ±ÈÝÒ×Òý·¢Õðµ´£©×÷Îª½ÇËÙ¶È¿ØÖÆÆ÷ÆÚÍûÖµ£¨target_rate£©¡£target_rate
ÓëÍÓÂÝÒÇµÃµ½µÄµ±Ç°½ÇËÙ¶È×÷²î£¬µÃµ½½ÇËÙ¶ÈÎó²î£¨rate_error£©³ËÒÔkpµÃµ½P¡£ÔÚIÖµÐ¡ÓÚÏÞ·ùÖµ£¨Õâ¸öÖµ´ó¸ÅÔÚ5%ÓÍÃÅ£©»òÕß
rate_errorÓëiÖµÒìºÅÊ±½«rate_errorÀÛ¼Óµ½IÖÐ¡£Ç°ºóÁ½´Îrate_errorµÄ²î×÷ÎªDÏî£¬ÖµµÃ×¢ÒâµÄÊÇ¼ÓÐèÒªÈë20hz
£¨Ò²¿ÉÒÔ²ÉÓÃÆäËüºÏÊÊÆµÂÊ£©ÂË²¨£¬ÒÔ±ÜÃâÕðµ´¡£½«P,I,DÈýÕßÏà¼Ó²¢ÏÞ·ù£¨50%ÓÍÃÅ£©µÃµ½×îÖÕPIDÊä³ö¡£
*/

//´®»·PIDµ÷½ÚÏêÇé²Î¼û£ºhttp://blog.csdn.net/super_mic ... 36723

//·ÉÐÐÆ÷×ËÌ¬Íâ»·¿ØÖÆ
void control::attitude_outter_loop(void)
{
    int32_t errorAngle[2];
    Vector3f Gyro_ADC;

    //¼ÆËã½Ç¶ÈÎó²îÖµ, ½Ç¶ÈÎó²îÖµ=ÆÚÍûÖµ-´Ë¿Ì×ËÌ¬Öµ
    //constrain_int32×÷ÓÃ£º32Î»ÕûÐÍÊýÏÞ·ù£¬Ê¹Æä¿ØÖÆÊäÈëµÄ×î´ó·ÉÐÐÇã½Ç²»´óÓÚ25¶È£¨Èç¹û¿ØÖÆÁ¿±È25¶È´ó£¬·É»úÔç¾Í×¹»ÙÁË£©
    //rc.Command[roll]£ºÒ£¿ØÊý¾Ý imu.angle.x £º´Ë¿Ì×ËÌ¬(½Ç¶È)
    //1.µÃµ½Öá×ËÌ¬µÄ½Ç¶È²î£¨errorAngle£©
    //2.Õâ¸ö½Ç¶È²îÖµ½øÐÐÏÞ·ù(constrain_int32)£¨Õý¸ºFLYANGLE_MAX£©
    //£¨ÏÞ·ù±ØÐëÓÐ£¬·ñÔò¾çÁÒ´ò¶æÊ±ÈÝÒ×Òý·¢Õðµ´£©×÷Îª½ÇËÙ¶È¿ØÖÆÆ÷ÆÚÍûÖµ£¨target_rate£©
    errorAngle[roll] = constrain_int32((rc.Command[roll] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10;
    errorAngle[pitch] = constrain_int32((rc.Command[pitch] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10;

    //»ñÈ¡´ËÊ±ÍÓÂÝÒÇÉÏµÄ½ÇËÙ¶È£¬È¡½ÇËÙ¶ÈµÄËÄ´ÎÆ½¾ùÖµ
    Gyro_ADC = mpu6050.Get_Gyro() / 4;
    /*
    µÃµ½Íâ»·PIDÊä³ö£¨½ÇËÙ¶ÈµÄ²îÖµ£©(ÊµÖÊÊÇÏàµ±ÓÚÄÚ»·µÄP±ÈÀýÏî)-------->
    3.target_rateÓëÍÓÂÝÒÇµÃµ½µÄµ±Ç°½ÇËÙ¶È×÷²î£¬µÃµ½½ÇËÙ¶ÈÎó²î£¨RateError£©³ËÒÔkp£¨Íâ»·¿ØÖÆÏµÊý pid[PIDLEVEL]--->(280, 0, 0
    , 0)£©µÃµ½¸øÄÚ»·µÄP¡£
    */

    //ºá¹öroll£ºÍâ»·¿ØÖÆ¡£ÊäÈëÎª½Ç¶È,Êä³öÎª½ÇËÙ¶È¡£RateError[roll] ×÷ÎªÄÚ»·µÄÊäÈë¡£
    RateError[roll] = pid[PIDLEVEL].get_p(errorAngle[roll]) - Gyro_ADC.x; //Gyro_ADC.x:ÍÓÂÝÒÇXÖáµÄÖµ
    //¸©Ñöpitch£ºÍâ»·¿ØÖÆ¡£ÊäÈëÎª½Ç¶È,Êä³öÎª½ÇËÙ¶È¡£RateError[pitch] ×÷ÎªÄÚ»·µÄÊäÈë¡£
    RateError[pitch] = pid[PIDLEVEL].get_p(errorAngle[pitch]) - Gyro_ADC.y;//Gyro_ADC.y:ÍÓÂÝÒÇYÖáµÄÖµ

    /*
    Æ«º½£¨Yaw£©µÄ¿ØÖÆËã·¨ºÍÇ°Á½ÕßÂÔÓÐ²»Í¬£¬ÊÇ½«´ò¶æÁ¿£¨Ò£¿ØÊý¾ÝÁ¿rc.Command[yaw]£©ºÍ½Ç¶ÈÎó²îµÄºÍ×÷Îª½ÇËÙ¶ÈÄÚ»·µÄÆÚÍûÖµ£¬
    ÕâÑù¿ÉÒÔ»ñµÃ¸üºÃµÄ¶¯Ì¬ÏìÓ¦¡£½ÇËÙ¶ÈÄÚ»·ºÍºá¹öÓë¸©ÑöµÄ¿ØÖÆ·½·¨Ò»ÖÂ£¬²ÎÊý£¨»ý·ÖÏÞ·ùÖµ»áºÜÐ¡£¬Ä¬ÈÏÖ»ÓÐÍò·ÖÖ®8£©ÉÏÓÐ²»Í¬¡£*/

    //º½Ïòyaw£ºÍâ»·¿ØÖÆ¡£ÊäÈëÎª½Ç¶È,Êä³öÎª½ÇËÙ¶È¡£ RateError[yaw] ×÷ÎªÄÚ»·µÄÊäÈë¡£
    RateError[yaw] = ((int32_t)(yawRate) * rc.Command[yaw]) / 32 - Gyro_ADC.z; //Gyro_ADC.z:ÍÓÂÝÒÇZÖáµÄÖµ

}

//·ÉÐÐÆ÷×ËÌ¬ÄÚ»·¿ØÖÆ: ÊäÈëÎª½ÇËÙ¶È£¬Êä³öÎªPWMÔöÁ¿
//ÄÚ»·µÄÐ§¹û¾ÍÊÇ£º¼õÐ¡ P±ÈÀý¿ØÖÆ´øÀ´µÄÕðµ´
void ANO_FlyControl::Attitude_Inner_Loop(void)
{
    int32_t PIDTerm[3];

    //×¢ÒâÕâÀïÊÇiµÄÖµÊÇ0µ½2
    //PIDROLL¡¢PIDPITCH¡¢PIDYAWÊÇÃ¶¾ÙÀàÐÍ£¬Ò²¾ÍÊÇ0¡¢1¡¢2£¬Ò²¾ÍÊÇÏÂÃæµÄpid ¡¢PIDTerm¾ÍÊÇ3×éPID
    for(u8 i=0; i<3;i++)
    {
        //ÏÖÏó£ºµ±ÓÍÃÅµÍÓÚ¼ì²éÖµÊ±»ý·ÖÇåÁã£¬ÖØÐÂ»ý·Ö

        //²Â²â£ºÕâÀïÓ¦¸ÃÊÇµ£ÐÄ·É»úÃ»·ÉÆðÀ´Ê±¾Í¿ªÊ¼ÓÐ»ý·Ö£¬»áµ¼ÖÂÆð·ÉÊ±²»ÎÈ¶¨
        if ((rc.rawData[THROTTLE]) < RC_MINCHECK)
            pid.reset_I();

        //get_pidº¯Êý£ºreturn get_p(error) + get_i(error, dt) + get_d(error, dt);-------->ÕâÀïÊµ¼Ê¾ÍÊÇÒ»¸öÍêÕûµÄPID
        //PID_INNER_LOOP_TIME£º2000us--->0.2ms »ý·ÖÎ¢·ÖÊ±¼ä£¬Ã¿¸ô0.2ms²Ù×÷»ý·ÖºÍÎ¢·Ö,RateErrorÊÇÍâ»·¼ÆËãµÄ½á¹û£¨´ÓÍâ»·Ëã³ö£©
        //µÃµ½ÄÚ»·PIDÊä³ö£¬Ö±½ÓÊä³ö×ªÎªµç»ú¿ØÖÆÁ¿
        PIDTerm = pid.get_pid(RateError, PID_INNER_LOOP_TIME);
    }

    //¶ÔYAW½Ç¼ÌÐø´¦Àí£¬¼ÓÈëÒ£¿Ø¿ØÖÆ
    //ÔÚIÖµÐ¡ÓÚÏÞ·ùÖµ£¨Õâ¸öÖµ´ó¸ÅÔÚ5%ÓÍÃÅ£©»òÕßrate_errorÓëiÖµÒìºÅÊ±½«rate_errorÀÛ¼Óµ½IÖÐ¡£
    PIDTerm[yaw] = -constrain_int32(PIDTerm[yaw], -300 - abs(rc.Command[yaw]), +300 + abs(rc.Command[yaw]));

    //PIDÊä³ö×ªÎªµç»ú¿ØÖÆÁ¿
    motor.writeMotor(rc.Command[THROTTLE], PIDTerm[roll], PIDTerm[pitch], PIDTerm[yaw]);
}

/*
¡¾µ÷½Ú´®»·PID´ó¸Å¹ý³Ì£¨×¢ÒâÐÞÕý·´Ïò£©¡¿

1¡¢¹À¼Æ´ó¸ÅµÄÆð·ÉÓÍÃÅ¡£
2¡¢µ÷Õû½ÇËÙ¶ÈÄÚ»·²ÎÊý¡£
3¡¢½«½Ç¶ÈÍâ»·¼ÓÉÏ£¬µ÷ÕûÍâ»·²ÎÊý¡£
4¡¢ºá¹ö¸©Ñö²ÎÊýÒ»°ã¿ÉÈ¡Ò»ÖÂ£¬½«·É»ú½â°ó£¬×¥ÔÚÊÖÖÐ²âÊÔÁ½¸öÖá»ìºÏ¿ØÖÆµÄÐ§¹û£¨×¢Òâ°²È«£©£¬ÓÐÎÊÌâ»Øµ½¡°¿¾ËÄÖá¡±¼ÌÐøµ÷Õû£¬Ö±Ö
Á·É»úÔÚÊÖÖÐ²»»á³é´¤¡£
5¡¢´ó¸ÅÉèÖÃÆ«º½²ÎÊý£¨²»×·Çó¶¯Ì¬ÏìÓ¦£¬Æð·ÉºóÍ·²»Æ«¼´¿É£©£¬Æð·ÉºóÔÙ¹Û²ìºá¹öºÍ¸©ÑöÖáÏò´ò¶æµÄ·´Ó¦£¬ÈçÓÐÎÊÌâ»Øµ½¡°¿¾ËÄÖá¡±¡£
6¡¢ºá¹öºÍ¸©ÑöokÒÔºó£¬ÔÙµ÷ÕûÆ«º½Öá²ÎÊýÒÔ´ïµ½ºÃµÄ¶¯Ì¬Ð§¹û¡£
*/

/*
¡¾¹ý³ÌÏê½â¡¿

1¡¢ÒªÔÚ·É»úµÄÆð·ÉÓÍÃÅ»ù´¡ÉÏ½øÐÐPID²ÎÊýµÄµ÷Õû£¬·ñÔò¡°¿¾ËÄÖá¡±µÄÊ±ºòµ÷ÊÔÎÈ¶¨ÁË£¬·ÉÆðÀ´ºÜ¿ÉÄÜÓÖ»á»Îµ´¡£
2¡¢ÄÚ»·µÄ²ÎÊý×îÎª¹Ø¼ü£¡ÀíÏëµÄÄÚ»·²ÎÊýÄÜ¹»ºÜºÃµØ¸úËæ´ò¶æ£¨½ÇËÙ¶È¿ØÖÆÄ£Ê½ÏÂµÄ´ò¶æ£©¿ØÖÆÁ¿¡£
ÔÚÆ½ºâÎ»ÖÃ¸½½ü£¨Õý¸º30¶È×óÓÒ£©£¬¶æÁ¿Í»¼Ó£¬·É»ú¿ìËÙÏìÓ¦£»¶æÁ¿»ØÖÐ£¬·É»úÁ¢¿ÌÍ£Ö¹ÔË¶¯£¨¼¸ºõÃ»ÓÐ»Øµ¯ºÍÕðµ´£©¡£
2.1Ê×ÏÈ¸Ä±ä³ÌÐò£¬½«½Ç¶ÈÍâ»·È¥µô£¬½«´ò¶æÁ¿×÷ÎªÄÚ»·µÄÆÚÍû£¨½ÇËÙ¶ÈÄ£Ê½£¬ÔÚAPMÖÐ½ÐACROÄ£Ê½£¬ÔÚ´ó½®ÖÐ½ÐÊÖ¶¯Ä£Ê½£©¡£
2.2¼ÓÉÏP£¬PÌ«Ð¡£¬²»ÄÜÐÞÕý½ÇËÙ¶ÈÎó²î±íÏÖÎªºÜ¡°Èí¡±ÇãÐ±ºóÄÑÒÔÐÞÕý£¬´ò¶æÏìÓ¦Ò²²î¡£PÌ«´ó£¬ÔÚÆ½ºâÎ»ÖÃÈÝÒ×Õðµ´£¬
´ò¶æ»ØÖÐ»ò¸ø¸ÉÈÅ£¨ÓÃÊÖÍ»¼Ó¸ÉÈÅ£©Ê±»áÕðµ´¡£ºÏÊÊµÄP
ÄÜ½ÏºÃµÄ¶Ô´ò¶æ½øÐÐÏìÓ¦£¬ÓÖ²»Ì«»áÕðµ´£¬µ«ÊÇ¶æÁ¿»ØÖÐºó»á»Øµ¯ºÃ¼¸ÏÂ²ÅÄÜÍ£Ö¹£¨Ã»ÓÐD£©¡£
2.3¼ÓÉÏD£¬DµÄÐ§¹ûÊ®·ÖÃ÷ÏÔ£¬¼Ó¿ì´ò¶æÏìÓ¦£¬×î´óµÄ×÷ÓÃÊÇÄÜºÜºÃµØÒÖÖÆ¶æÁ¿»ØÖÐºóµÄÕðµ´£¬¿ÉÎ½Á¢¸Í¼ûÓ°¡£
Ì«´óµÄD»áÔÚºá¹ö¸©Ñö»ì¿ØÊ±±íÏÖ³öÀ´£¨¾¡¹ÜÔÚ¡°¿¾ËÄÖá¡±Ê±µÄ±íÏÖ¿ÉÄÜºÜºÃ£©£¬¾ßÌå±íÏÖÊÇËÄÖá×¥ÔÚÊÖÀïÍÆÓÍÃÅ»á³é´¤¡£
Èç¹ûÕâÑù£¬Ö»ÄÜ»Øµ½¡°¿¾ËÄÖá¡±½µµÍD£¬Í¬Ê±PÒ²Ö»ÄÜ¸ú×Å½µµÍ¡£Dµ÷ÕûÍêºó¿ÉÒÔÔÙ´Î¼Ó´óPÖµ£¬ÒÔÄÜ¹»¸úËæ´ò¶æÎªÅÐ¶Ï±ê×¼¡£
2.4¼ÓÉÏI£¬»á·¢ÏÖÊÖ¸Ð±äµÃÈáºÍÁËÐ©¡£ÓÉÓÚ±ÊÕß¡°¿¾ËÄÖá¡±µÄ×°ÖÃÖÐËÄÖáµÄÖØÐÄ¸ßÓÚÐý×ªÖá£¬Õâ¾ö¶¨ÁËÔÚËÄÖáÆ«ÀëË®Æ½Î»ÖÃºó
»áÓÐÖØÁ¦·ÖÁ¿Ê¹µÃËÄÖá»á¼ÌÐøÆ«ÀëÆ½ºâÎ»ÖÃ¡£IµÄ×÷ÓÃ¾Í¿ÉÒÔÊ¹µÃÔÚÒ»¶¨½Ç¶È·¶Î§ÄÚ£¨30¶È×óÓÒ£©¿ÉÒÔÐÞÕýÖØÁ¦´øÀ´µÄÓ°Ïì¡£
±íÏÖ´ò¶æÊ¹µÃ·É»úÆ«ÀëÆ½ºâÎ»ÖÃ£¬¶æÁ¿»ØÖÐºó·É»úÁ¢¿ÌÍ£Ö¹×ª¶¯£¬ÈôÃ»ÓÐI»òÌ«Ð¡£¬·É»ú»áÓÉÓÚÖØÁ¦¼ÌÐø×ª¶¯¡£

3¡¢½Ç¶ÈÍâ»·Ö»ÓÐÒ»¸ö²ÎÊýP¡£½«Íâ»·¼ÓÉÏ£¨ÔÚAPMÖÐ½ÐStabilizeÄ£Ê½£¬ÔÚ´ó½®ÖÐ½Ð×ËÌ¬Ä£Ê½£©¡£´ò¶æ»á¶ÔÓ¦µ½ÆÚÍûµÄ½Ç¶È¡£
PµÄ²ÎÊý±È½Ï¼òµ¥¡£Ì«Ð¡£¬´ò¶æ²»ÁéÃô£¬Ì«´ó£¬´ò¶æ»ØÖÐÒ×Õðµ´¡£ÒÔºÏÊÊµÄ´ò¶æ·´Ó¦ËÙ¶ÈÎª×¼¡£

4¡¢ÖÁ´Ë£¬¿¾ËÄÖá¡±Ð§¹ûÓ¦¸Ã»áºÜºÃÁË£¬µ«ÊÇÁ½¸öÖá»ì¿ØµÄÐ§¹ûÈçºÎ»¹²»Ò»¶¨£¬ÓÐ¿ÉÄÜ»á³é£¨Á½¸öÖáµÄ¿ØÖÆÁ¿µþ¼ÓÆðÀ´£¬
ÌØ±ðÊÇ½Ï´óµÄD£¬»áÒýÆð³é´¤£©¡£Èç¹û³éÁË£¬½µµÍPDµÄÖµ£¬I»ù±¾²»ÓÃ±ä¡£

5¡¢¼ÓÉÏÆ«º½µÄÐÞÕý²ÎÊýºó£¨Ö±½Ó¸øË«»·²ÎÊý£¬½Ç¶ÈÍâ»·PºÍºá¹ö²î²»¶à£¬ÄÚ»·P±Èºá¹ö´óÐ©£¬IºÍºá¹ö²î²»¶à£¬D¿ÉÒÔÏÈ²»¼Ó£©£¬
ÄÃÔÚÊÖÉÏÊÔ¹ýÐÞÕýºÍ´ò¶æ·½ÏòÕýÈ·ºó¿ÉÒÔÊÔ·ÉÁË£¨ÊÔ·ÉºÜÎ£ÏÕ£¡£¡£¡£¡Ñ¡ÔñÔÚ¿í³¨¡¢ÎÞ·çµÄÊÒÄÚ£¬1
Ã×µÄ¸ß¶È£¨¸ß¶ÈÌ«µÍ»áÓÐµØÃæÐ§Ó¦¸ÉÈÅ£¬
Ì«¸ß²»ÈÝÒ×¿´Çå×ËÌ¬ÇÒÈÝÒ×Ë¤»µ£©£¬±Ü¿ªÈËÈºµÄµØ·½±È½ÏÊÊºÏ£¬ÈçÓÐÒâÍâÇé¿ö£¬Á¢¿Ì¹Ø±ÕÓÍÃÅ£¡£¡£¡
5.1ÊÔ·ÉÊ±Ö÷Òª¹Û²ìÕâÃ´¼¸¸ö·½ÃæµÄÇé¿ö£¬Ò»°ã¾­¹ýµ÷ÕûµÄ²ÎÊýÔÚÆ½ºâÎ»ÖÃ²»»á´ó·ù¶ÈÕðµ´£¬ÐèÒª¹Û²ì£º
5.1.1ÔÚÆ½ºâÎ»ÖÃÓÐÃ»ÓÐÐ¡·ù¶ÈÕðµ´£¨¿ÉÄÜÊÇÓÉÓÚ»ú¼ÜÕð¶¯Ì«´óµ¼ÖÂ×ËÌ¬½âËã´íÎóÔì³É¡£Ò²¿ÉÄÜÊÇ½ÇËÙ¶ÈÄÚ»·DµÄ²¨¶¯¹ý´ó£¬
Ç°Õß¿ÉÒÔ¼ÓÇ¿¼õÕð´ëÊ©£¬´«¸ÐÆ÷ÏÂÌùÉÏ3M½º£¬±ØÒªÊ±ÔÚÁ½²ã3MÅÝÄ­½ºÖÐ¼ÐÉÏ¡°¼õÕð°å¡±£¬×¢Òâ£ºÌú´ÅÐÔµÄ¼õÕð°å»á¸ÉÈÅ´ÅÁ¦¼Æ¶ÁÊý£»
ºóÕß¿ÉÒÔ³¢ÊÔ½µµÍDÏîÂË²¨µÄ½ØÖ¹ÆµÂÊ£©¡£
5.1.2¹Û²ì´ò¶æÏìÓ¦µÄËÙ¶ÈºÍ¶æÁ¿»ØÖÐºó·É»úµÄ»Ø¸´ËÙ¶È¡£
5.1.3¸÷¸ö·½Ïò£¨¼ÇµÃ²âÊÔÓÒÇ°£¬×óºóµÈ·½Ïò£©´ó¶æÁ¿Í»¼ÓÊäÈë²¢»ØÖÐÊ±ÊÇ·ñ»áÒýÆðÕðµ´¡£
ÈçÓÐ£¬³¢ÊÔ¼õÐ¡ÄÚ»·PDÒ²¿ÉÄÜÊÇÓÉÓÚ¡°ÓÒÇ°¡±µÈ»ì¿Ø·½ÏòÉÏµÄ¶æÁ¿Ì«´óÔì³É¡£

6¡¢ºá¹öºÍ¸©Ñöµ÷ºÃºó¾Í¿ÉÒÔµ÷ÕûÆ«º½µÄ²ÎÊýÁË¡£ºÏÊÊ²ÎÊýµÄÅÐ¶Ï±ê×¼ºÍÖ®Ç°Ò»Ñù£¬´ò¶æ¿ìËÙÏìÓ¦£¬¶æÁ¿»ØÖÐ·É»úÁ¢¿ÌÍ£Ö¹×ª¶¯£¨²ÎÊýD
µÄ×÷ÓÃ£©¡£

ÖÁ´Ë£¬Ë«»·PID²ÎÊýµ÷½ÚÍê±Ï£¡×£Ë¬·É£¡
*/