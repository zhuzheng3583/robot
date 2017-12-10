#include "imu.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

//本文将分析一种常见的四轴飞行器姿态解算方法，Mahony的互补滤波法。
//此法简单有效，希望能给学习四轴飞行器的朋友们带来帮助。
//关于姿态解算和滤波的理论知识，推荐秦永元的两本书，
//一是《惯性导航》，目前已出到第二版了;二是《卡尔曼滤波与组合导航原理》。
//程序中的理论基础，可在书中寻找。

//先定义Kp，Ki，以及halfT 。
//Kp，Ki，控制加速度计修正陀螺仪积分姿态的速度
//halfT ，姿态解算时间的一半。此处解算姿态速度为500HZ，因此halfT 为0.001
#define Kp 15.0f //1.6f //2.0f
#define Ki 0.0001f//0.000000001f //0.001f //0.002f
#define halfT 0.0025f
//初始化四元数
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
//定义姿态解算误差的积分
float exInt = 0, eyInt = 0, ezInt = 0;

//以下为姿态解算函数。
//参数gx，gy，gz分别对应三个轴的角速度，单位是弧度/秒;
//参数ax，ay，az分别对应三个轴的加速度原始数据
//由于加速度的噪声较大，此处应采用滤波后的数据
void imu_update(float gx, float gy, float gz, float ax, float ay, float az, struct vehicle_attitude_s *att)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    //将加速度的原始数据，归一化，得到单位加速度
    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    //把四元数换算成“方向余弦矩阵”中的第三列的三个元素。根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。所以这里的vx、vy、vz，其实就是当前的机体坐标参照系上，换算出来的重力单位向量。(用表示机体姿态的四元数进行换算)
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    //这里说明一点，加速度计由于噪声比较大，而且在飞行过程中，受机体振动影响比陀螺仪明显，短时间内的可靠性不高。陀螺仪噪声小，但是由于积分是离散的，长时间的积分会出现漂移的情况，因此需要将用加速度计求得的姿态来矫正陀螺仪积分姿态的漂移。
    //在机体坐标参照系上，加速度计测出来的重力向量是ax、ay、az;陀螺积分后的姿态来推算出的重力向量是vx、vy、vz;它们之间的误差向量，就是陀螺积分后的姿态和加速度计测出来的姿态之间的误差。
    //向量间的误差，可以用向量积(也叫外积、叉乘)来表示，ex、ey、ez就是两个重力向量的叉积。这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
    //叉乘是数学基础，百度百科里有详细解释。
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    //将叉乘误差进行积分
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
    //用叉乘误差来做PI修正陀螺零偏，通过调节Kp，Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;
    //四元数微分方程，没啥好说的了，看上面推荐的书吧，都是理论的东西，自个琢磨琢磨
    //实在琢磨不明白，那就把指定的参数传进这个函数，再得到相应的四元数，最后转化成欧拉角即可了。不过建议还是把理论弄清楚一点。
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
    //四元数单位化
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

	//att->yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 57.3;
	//匿名四轴
    //angle->yaw += gyr->Z*Gyro_G*0.002f;
	//angle->rol = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 - AngleOffset_Pit; // pitch
	//angle->pit = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 - AngleOffset_Rol; // roll

	//某论坛
    //att->roll = atan2(2*(q1*q2 + 2*q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)*57.3;
    //att->pitch = asin(-2*(q1*q3 + 2*q0* q2))*57.3;
    //att->yaw = atan2f(2*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3)*57.3;

	//原子开发板MPU6050实验
    att->roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3;
    att->pitch = asin(-2*q1*q3 + 2*q0*q2)*57.3;
    att->yaw = atan2f(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)*57.3;

    att->rollspeed = gx;
    att->pitchspeed = gy;
    att->yawspeed = gz;

    att->rollacc = ax;
	att->pitchacc = ay;
	att->yawacc = az;
	// ==>
	// (z)
	// X-----> (x)
	// |
	// |
	// \/ (y)
}

#ifdef __cplusplus
}
#endif

