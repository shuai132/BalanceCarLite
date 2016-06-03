
#include "kalman.h"

/*
 * kalman平滑滤波器
 * 作者:刘帅
 * 2016-06-03
 * r参数调整滤波后的曲线与实测曲线的相近程度，r越小越接近。
 * q参数调滤波后的曲线平滑程度，q越小越平滑。
 */
float kalman(float y)
{
    static float x = 1;
    static float p = 5;
    static float q = 0.2; //important
    static float r = 1;   //important
    static float kg;

    p = p + q;
    kg = p / (p + r);
    x = x + kg * (y - x);
    p = (1 - kg) * p;
    return x;
}


//*********************************************************
// 卡尔曼滤波
//*********************************************************
//在程序中利用Angle+=(Gyro - Q_bias) * dt计算出陀螺仪积分出的角度，其中Q_bias是陀螺仪偏差。
//此时利用陀螺仪积分求出的Angle相当于系统的估计值，得到系统的观测方程；而加速度计检测的角度Accel相当于系统中的测量值，得到系统状态方程。
//程序中Q_angle和Q_gyro分别表示系统对加速度计及陀螺仪的信任度。根据Pdot = A*P + P*A' + Q_angle计算出先验估计协方差的微分，用于将当前估计值进行线性化处理。其中A为雅克比矩阵。
//随后计算系统预测角度的协方差矩阵P。计算估计值Accel与预测值Angle间的误差Angle_err。
//计算卡尔曼增益K_0,K_1，K_0用于最优估计值，K_1用于计算最优估计值的偏差并更新协方差矩阵P。
//通过卡尔曼增益计算出最优估计值Angle及预测值偏差Q_bias，此时得到最优角度值Angle及角速度值。
//Kalman滤波，20MHz的处理时间约0.77ms（STC12C5A60S2）

//******卡尔曼参数************
static float Q_angle = 0.001;//角度数据置信度
static float Q_gyro  = 0.005;//角速度数据置信度
static float R_angle = 0.5;
static float dt  = 0.004;    //dt为kalman滤波器采样时间;
static char  C_0 = 1;
static float Q_bias, Angle_err;
static float PCt_0, PCt_1, E;
static float K_0, K_1, t_0, t_1;
static float Pdot[4]  = {0, 0, 0, 0};
static float PP[2][2] = { { 1, 0 }, { 0, 1 } };
//外部要用的角度和角速度
float Angle;
float Gyro_y;

void Kalman_Filter(float Accel, float Gyro)
{
    Angle += (Gyro - Q_bias) * dt; //先验估计
    Angle_err = Accel - Angle;     //zk-先验估计

    // Pk-先验估计误差协方差的微分
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] =  Q_gyro;

    // Pk-先验估计误差协方差微分的积分=先验估计误差协方差
    PP[0][0] += Pdot[0] * dt;
    PP[0][1] += Pdot[1] * dt;
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    // 后验估计误差协方差
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle    += K_0 * Angle_err;   //后验估计  (最优角度)
    Q_bias   += K_1 * Angle_err;   //后验估计
    Gyro_y   =  Gyro - Q_bias;     //输出值(后验估计)的微分=角速度  (最优角速度)
}

