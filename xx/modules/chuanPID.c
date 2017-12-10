#include "CONTROL.h"
#include "IMU1.h"
#include "moto.h"
#include "RFdate.h"
#include <math.h>
extern T_RC_Data                         rc_d;                //ң��ͨ������;

extern u8 txbuf[4];         //���ͻ���
extern u8 rxbuf[4];         //���ջ���
extern u16 test1[3]; //���յ�NRf24L01����
extern S_INT16_XYZ ACC_F,GYRO_F;

PID PID_ROL,PID_PIT,PID_YAW;

extern S_INT16_XYZ        MPU6050_ACC_LAST,MPU6050_GYRO_LAST;


int Motor_Ele=0;                                           //��������
int Motor_Ail=0;                                           //�������

//u8 ARMED = 0;

//float rol_i=0,pit_i=0,yaw_p=0;
float thr=0;

S_FLOAT_XYZ EXP_ANGLE ,DIF_ANGLE;
PID1 PID_Motor;
/*********************************/
float pitch_i,roll_i,Yaw_i;                         //������
float pitch_old,roll_old,yaw_old;                   //�Ƕȱ���
float pitch_d,roll_d,yaw_d;                         //΢����
float rc_pitch,rc_roll,rc_yaw;                      //��̬��

//�⻷PID����
float pitch_shell_kp=280;//30 140
float pitch_shell_kd=0;//
float pitch_shell_ki=0;//
float roll_shell_kp=250;//30
float roll_shell_kd=0;//10
float roll_shell_ki=0;//0.08
float yaw_shell_kp=1.5;//10;//30
float yaw_shell_kd=0;//10
float yaw_shell_ki=0;//0.08;//0.08
float pitch_shell_out,roll_shell_out,yaw_shell_out; //�⻷�����

//�ڻ�PID����
float pitch_core_kp=0.040;
float Pitch_core_kd=0.002;////0.007;//0.07;0.008;
float Roll_core_kp=0.040;//;
float Roll_core_kd=0.002;////0.007;//06;//0.07;
float Yaw_core_kp=0.046;//;
float Yaw_core_kd=0.012;////0.007;//06;//0.07;
float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//�����Ǳ���
float pitch_core_kp_out,pitch_core_kd_out,Roll_core_kp_out,Roll_core_kd_out,yaw_core_kp_out,Yaw_core_kd_out;//�ڻ��������
float pitch_core_out,Roll_core_out,Yaw_core_out;//�ڻ������

int16_t moto1=0,moto2=0,moto3=0,moto4=0;

float tempjd=0;
void CONTROL(float rol, float pit, float yaw)
{
    ////////////////////////�⻷�ǶȻ�(PID)///////////////////////////////
    rc_pitch = (rc_d.pitch - 1500) / 20;
    pitch_i += (q_angle.pitch - rc_pitch);
    //-------------Pitch�����޷�----------------//
    if(pitch_i > 300) pitch_i = 300;
    else if(pitch_i < -300) pitch_i = -300;
    //-------------Pitch΢��--------------------//
    pitch_d = q_angle.pitch - pitch_old;
    //-------------pitch  PID-------------------//
    pitch_shell_out = pitch_shell_kp*(q_angle.pitch - rc_pitch) + pitch_shell_ki*pitch_i + pitch_shell_kd*pitch_d;
    //�Ƕȱ���
    pitch_old = q_angle.pitch;

    rc_roll = (rc_d.roll - 1500) / 20;
    roll_i += (q_angle.roll - rc_roll);
    //-------------Roll�����޷�----------------//
    if(roll_i > 300) roll_i = 300;
    else if(roll_i < -300) roll_i = -300;
    //-------------Roll΢��--------------------//
    roll_d = q_angle.roll - roll_old;
    //-------------Roll  PID-------------------//
    roll_shell_out = roll_shell_kp*(q_angle.roll - rc_roll) + roll_shell_ki*roll_i + roll_shell_kd*roll_d;
    //------------Roll�Ƕȱ���------------------//
    roll_old = q_angle.roll;


    rc_yaw=(rc_d.yaw-1500)*10;
    //-------------Yaw΢��--------------------//
    yaw_d=MPU6050_GYRO_LAST.Z - yaw_old;
    //-------------Roll  PID-------------------//
    yaw_shell_out  = yaw_shell_kp*(MPU6050_GYRO_LAST.Z-rc_yaw) + yaw_shell_ki*Yaw_i + yaw_shell_kd*yaw_d;
    //------------Roll�Ƕȱ���------------------//
    yaw_old=MPU6050_GYRO_LAST.Z;

    ////////////////////////�ڻ����ٶȻ�(PD)///////////////////////////////
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
    Gyro_radian_old_z = MPU6050_GYRO_LAST.Z;   //������ʷֵ

//--------------------�����ֵ�ںϵ��ĸ����--------------------------------//


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
* �ļ��� ��ANO_FlyControl.cpp
* ���� �����п���
**********************************************************************************/
include "ANO_FlyControl.h"ANO_FlyControl fc;

/*
�������ڻ����������⻷��
������������ѣ���С����˳���
���Ǳ�������֣�����ٰ�΢�ּ�
�����񵴺�Ƶ������������Ҫ�Ŵ�
����Ư���ƴ��壬����������С��
����ƫ��ظ���������ʱ�����½�
���߲������ڳ�������ʱ���ټӳ�
������Ƶ�ʿ죬�Ȱ�΢�ֽ�����
���������������΢��ʱ��Ӧ�ӳ�
����������������ǰ�ߺ��4��1
*/

/*
ROLL��PIT���������Ϲ�ʽ����PID�������YAW��Ƚ����⣬��Ϊƫ���Ƿ��߷���պú͵�������ƽ�У�
�������ĽǶ��޷��ɼ��ٶȼ�ֱ�Ӳ�ã���Ҫ����һ������������������ٶȼơ������ʹ�����̵Ļ���
���ǿ��Ե�����ͨ�����ٶȻ��������ƫ���ǣ�ȱ�������ڻ��ֻ����д��ڻ���Ư�ƣ�ƫ��������ʱ�������
��ƫ��Խ��Խ�����ǲ�ʹ�����̾�û�б����ֻ��ʹ��΢�ֻ��������ơ�
*/

ANO_FlyControl::ANO_FlyControl()
{
    yawRate = 120;
    //����PID����
    PID_Reset();
}

//����PID����
void ANO_FlyControl:ID_Reset(void)
{
    //��ΪYAW�ǶȻ�Ư�ƣ����Բ�����ROLL��PITCH��һ��
    pid[PIDROLL].set_pid(70, 15, 120, 2000000); //ROLL�Ƕȵ��ڻ�����ϵ��,20000:��������
    pid[PIDPITCH].set_pid(70, 30, 120, 2000000);//PITCH�Ƕȵ��ڻ�����ϵ��
    pid[PIDYAW].set_pid(100, 50, 0, 2000000); //YAW�Ƕȵ��ڻ�����ϵ��

    pid[PIDLEVEL].set_pid(280, 0, 0, 0); //�⻷����ϵ��
    pid[PIDMAG].set_pid(15, 0, 0, 0); //�������̿���ϵ��
}

/*
��ɨä֪ʶ��
����PID�����õĽǶ�P�ͽ��ٶ�PID��˫�ջ�PID�㷨------>�Ƕȵ�����Ϊ�������뵽���ٶȿ������� ���Ƕȵ�΢�־��ǽ��ٶȣ�
���ڱ�ϵͳ������˽��Ƕȿ�������ٶȿ��Ƽ����ķ�ʽ����������� PID ��������

���� PID �㷨�У����ٶ��ڻ�ռ�ż�Ϊ��Ҫ�ĵ�λ���ڶ���������е�����ģ�ͽ�
�з����󣬿���֪�����ϵͳ���ȶ����������֮һ���ǲ��ȶ��Ľ��ٶȡ�
��ˣ����ܹ�ֱ�Ӷ�ϵͳ�Ľ��ٶȽ��нϺõıջ����ƣ���Ȼ�����ϵͳ�Ķ�̬����
�����ȶ��ԣ�ͨ��Ҳ�ѽ��ٶ��ڻ���Ϊ���Ȼ��ڡ����Ƕ��⻷�������������ڶ��������
��������̬�ǵľ�ȷ���ơ�
�⻷������Ϊ�Ƕ�,���Ϊ���ٶ�
�ڻ�������Ϊ���ٶȣ����ΪPWM����
ʹ�ô���pid����Ϊ���ǶȻ�����pid�����ͽ��ٶȿ��ƻ��ȶ���������Ϊ�ǶȻ����⻷��������Ϊ���ٶȻ����ڻ�����
��������ԭ��Ϊ���ں��⣬���������ڻ�ʱ���⻷��PID����Ϊ0
��ν�⻷����ֻ��һ��P�������ã�Ҳ���Ǳ����������ã�PҲ�����������ȣ�Խ��Խ����ʹ�ɻ��𵴡�
�𵴵��ص��ǣ�Ƶ��С�����ȴ�
*/

/*
�������Roll���͸�����Pitch���Ŀ����㷨��
�����Roll���͸�����Pitch���Ŀ����㷨��һ���ģ����Ʋ���Ҳ�ȽϽӽ���

���ȵõ�����̬�ĽǶȲangle error���������ֵ���ԽǶ�ϵ��p
���޷����޷������У�������Ҵ��ʱ���������𵴣���Ϊ���ٶȿ���������ֵ��target_rate����target_rate
�������ǵõ��ĵ�ǰ���ٶ�����õ����ٶ���rate_error������kp�õ�P����IֵС���޷�ֵ�����ֵ�����5%���ţ�����
rate_error��iֵ���ʱ��rate_error�ۼӵ�I�С�ǰ������rate_error�Ĳ���ΪD�ֵ��ע����Ǽ���Ҫ��20hz
��Ҳ���Բ�����������Ƶ�ʣ��˲����Ա����𵴡���P,I,D������Ӳ��޷���50%���ţ��õ�����PID�����
*/

//����PID��������μ���http://blog.csdn.net/super_mic ... 36723

//��������̬�⻷����
void control::attitude_outter_loop(void)
{
    int32_t errorAngle[2];
    Vector3f Gyro_ADC;

    //����Ƕ����ֵ, �Ƕ����ֵ=����ֵ-�˿���ֵ̬
    //constrain_int32���ã�32λ�������޷���ʹ������������������ǲ�����25�ȣ������������25�ȴ󣬷ɻ����׹���ˣ�
    //rc.Command[roll]��ң������ imu.angle.x ���˿���̬(�Ƕ�)
    //1.�õ�����̬�ĽǶȲerrorAngle��
    //2.����ǶȲ�ֵ�����޷�(constrain_int32)������FLYANGLE_MAX��
    //���޷������У�������Ҵ��ʱ���������𵴣���Ϊ���ٶȿ���������ֵ��target_rate��
    errorAngle[roll] = constrain_int32((rc.Command[roll] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10;
    errorAngle[pitch] = constrain_int32((rc.Command[pitch] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10;

    //��ȡ��ʱ�������ϵĽ��ٶȣ�ȡ���ٶȵ��Ĵ�ƽ��ֵ
    Gyro_ADC = mpu6050.Get_Gyro() / 4;
    /*
    �õ��⻷PID��������ٶȵĲ�ֵ��(ʵ�����൱���ڻ���P������)-------->
    3.target_rate�������ǵõ��ĵ�ǰ���ٶ�����õ����ٶ���RateError������kp���⻷����ϵ�� pid[PIDLEVEL]--->(280, 0, 0
    , 0)���õ����ڻ���P��
    */

    //���roll���⻷���ơ�����Ϊ�Ƕ�,���Ϊ���ٶȡ�RateError[roll] ��Ϊ�ڻ������롣
    RateError[roll] = pid[PIDLEVEL].get_p(errorAngle[roll]) - Gyro_ADC.x; //Gyro_ADC.x:������X���ֵ
    //����pitch���⻷���ơ�����Ϊ�Ƕ�,���Ϊ���ٶȡ�RateError[pitch] ��Ϊ�ڻ������롣
    RateError[pitch] = pid[PIDLEVEL].get_p(errorAngle[pitch]) - Gyro_ADC.y;//Gyro_ADC.y:������Y���ֵ

    /*
    ƫ����Yaw���Ŀ����㷨��ǰ�������в�ͬ���ǽ��������ң��������rc.Command[yaw]���ͽǶ����ĺ���Ϊ���ٶ��ڻ�������ֵ��
    �������Ի�ø��õĶ�̬��Ӧ�����ٶ��ڻ��ͺ���븩���Ŀ��Ʒ���һ�£������������޷�ֵ���С��Ĭ��ֻ�����֮8�����в�ͬ��*/

    //����yaw���⻷���ơ�����Ϊ�Ƕ�,���Ϊ���ٶȡ� RateError[yaw] ��Ϊ�ڻ������롣
    RateError[yaw] = ((int32_t)(yawRate) * rc.Command[yaw]) / 32 - Gyro_ADC.z; //Gyro_ADC.z:������Z���ֵ

}

//��������̬�ڻ�����: ����Ϊ���ٶȣ����ΪPWM����
//�ڻ���Ч�����ǣ���С P�������ƴ�������
void ANO_FlyControl::Attitude_Inner_Loop(void)
{
    int32_t PIDTerm[3];

    //ע��������i��ֵ��0��2
    //PIDROLL��PIDPITCH��PIDYAW��ö�����ͣ�Ҳ����0��1��2��Ҳ���������pid ��PIDTerm����3��PID
    for(u8 i=0; i<3;i++)
    {
        //���󣺵����ŵ��ڼ��ֵʱ�������㣬���»���

        //�²⣺����Ӧ���ǵ��ķɻ�û������ʱ�Ϳ�ʼ�л��֣��ᵼ�����ʱ���ȶ�
        if ((rc.rawData[THROTTLE]) < RC_MINCHECK)
            pid.reset_I();

        //get_pid������return get_p(error) + get_i(error, dt) + get_d(error, dt);-------->����ʵ�ʾ���һ��������PID
        //PID_INNER_LOOP_TIME��2000us--->0.2ms ����΢��ʱ�䣬ÿ��0.2ms�������ֺ�΢��,RateError���⻷����Ľ�������⻷�����
        //�õ��ڻ�PID�����ֱ�����תΪ���������
        PIDTerm = pid.get_pid(RateError, PID_INNER_LOOP_TIME);
    }

    //��YAW�Ǽ�����������ң�ؿ���
    //��IֵС���޷�ֵ�����ֵ�����5%���ţ�����rate_error��iֵ���ʱ��rate_error�ۼӵ�I�С�
    PIDTerm[yaw] = -constrain_int32(PIDTerm[yaw], -300 - abs(rc.Command[yaw]), +300 + abs(rc.Command[yaw]));

    //PID���תΪ���������
    motor.writeMotor(rc.Command[THROTTLE], PIDTerm[roll], PIDTerm[pitch], PIDTerm[yaw]);
}

/*
�����ڴ���PID��Ź��̣�ע���������򣩡�

1�����ƴ�ŵ�������š�
2���������ٶ��ڻ�������
3�����Ƕ��⻷���ϣ������⻷������
4�������������һ���ȡһ�£����ɻ����ץ�����в����������Ͽ��Ƶ�Ч����ע�ⰲȫ����������ص��������ᡱ����������ֱ�
��ɻ������в���鴤��
5���������ƫ����������׷��̬��Ӧ����ɺ�ͷ��ƫ���ɣ�����ɺ��ٹ۲����͸���������ķ�Ӧ����������ص��������ᡱ��
6������͸���ok�Ժ��ٵ���ƫ��������Դﵽ�õĶ�̬Ч����
*/

/*
��������⡿

1��Ҫ�ڷɻ���������Ż����Ͻ���PID�����ĵ��������򡰿����ᡱ��ʱ������ȶ��ˣ��������ܿ����ֻ�ε���
2���ڻ��Ĳ�����Ϊ�ؼ���������ڻ������ܹ��ܺõظ����棨���ٶȿ���ģʽ�µĴ�棩��������
��ƽ��λ�ø���������30�����ң�������ͻ�ӣ��ɻ�������Ӧ���������У��ɻ�����ֹͣ�˶�������û�лص����𵴣���
2.1���ȸı���򣬽��Ƕ��⻷ȥ�������������Ϊ�ڻ������������ٶ�ģʽ����APM�н�ACROģʽ���ڴ��н��ֶ�ģʽ����
2.2����P��P̫С�������������ٶ�������Ϊ�ܡ�����б�����������������ӦҲ�P̫����ƽ��λ�������𵴣�
�����л�����ţ�����ͻ�Ӹ��ţ�ʱ���𵴡����ʵ�P
�ܽϺõĶԴ�������Ӧ���ֲ�̫���𵴣����Ƕ������к��ص��ü��²���ֹͣ��û��D����
2.3����D��D��Ч��ʮ�����ԣ��ӿ�����Ӧ�������������ܺܺõ����ƶ������к���𵴣���ν���ͼ�Ӱ��
̫���D���ں���������ʱ���ֳ����������ڡ������ᡱʱ�ı��ֿ��ܺܺã����������������ץ�����������Ż�鴤��
���������ֻ�ܻص��������ᡱ����D��ͬʱPҲֻ�ܸ��Ž��͡�D�����������ٴμӴ�Pֵ�����ܹ�������Ϊ�жϱ�׼��
2.4����I���ᷢ���ָб�������Щ�����ڱ��ߡ������ᡱ��װ������������ĸ�����ת�ᣬ�������������ƫ��ˮƽλ�ú�
������������ʹ����������ƫ��ƽ��λ�á�I�����þͿ���ʹ����һ���Ƕȷ�Χ�ڣ�30�����ң�������������������Ӱ�졣
���ִ��ʹ�÷ɻ�ƫ��ƽ��λ�ã��������к�ɻ�����ֹͣת������û��I��̫С���ɻ���������������ת����

3���Ƕ��⻷ֻ��һ������P�����⻷���ϣ���APM�н�Stabilizeģʽ���ڴ��н���̬ģʽ���������Ӧ�������ĽǶȡ�
P�Ĳ����Ƚϼ򵥡�̫С����治������̫�󣬴��������𵴡��Ժ��ʵĴ�淴Ӧ�ٶ�Ϊ׼��

4�����ˣ������ᡱЧ��Ӧ�û�ܺ��ˣ������������ص�Ч����λ���һ�����п��ܻ�飨������Ŀ���������������
�ر��ǽϴ��D��������鴤����������ˣ�����PD��ֵ��I�������ñ䡣

5������ƫ��������������ֱ�Ӹ�˫���������Ƕ��⻷P�ͺ����࣬�ڻ�P�Ⱥ����Щ��I�ͺ����࣬D�����Ȳ��ӣ���
���������Թ������ʹ�淽����ȷ������Է��ˣ��Էɺ�Σ�գ�������ѡ���ڿ����޷�����ڣ�1
�׵ĸ߶ȣ��߶�̫�ͻ��е���ЧӦ���ţ�
̫�߲����׿�����̬������ˤ�������ܿ���Ⱥ�ĵط��Ƚ��ʺϣ�����������������̹ر����ţ�����
5.1�Է�ʱ��Ҫ�۲���ô��������������һ�㾭�������Ĳ�����ƽ��λ�ò��������𵴣���Ҫ�۲죺
5.1.1��ƽ��λ����û��С�����𵴣����������ڻ�����̫������̬���������ɡ�Ҳ�����ǽ��ٶ��ڻ�D�Ĳ�������
ǰ�߿��Լ�ǿ�����ʩ��������������3M������Ҫʱ������3M��ĭ���м��ϡ�����塱��ע�⣺�����Եļ�������Ŵ����ƶ�����
���߿��Գ��Խ���D���˲��Ľ�ֹƵ�ʣ���
5.1.2�۲�����Ӧ���ٶȺͶ������к�ɻ��Ļظ��ٶȡ�
5.1.3�������򣨼ǵò�����ǰ�����ȷ��򣩴����ͻ�����벢����ʱ�Ƿ�������𵴡�
���У����Լ�С�ڻ�PDҲ���������ڡ���ǰ���Ȼ�ط����ϵĶ���̫����ɡ�

6������͸������ú�Ϳ��Ե���ƫ���Ĳ����ˡ����ʲ������жϱ�׼��֮ǰһ������������Ӧ���������зɻ�����ֹͣת��������D
�����ã���

���ˣ�˫��PID����������ϣ�ףˬ�ɣ�
*/