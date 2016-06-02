#include "balancefilter.h"

//一阶互补滤波 (测试发现效果不太明显)
static float K1 = 0.05;       // 对加速度计取值的权重
float dt = 0.004;      //注意：dt的取值为滤波器采样时间
float angle1;

void Yijielvbo(float angle_m, float gyro_m)//采集后计算的角度和角加速度
{
    angle1 = K1 * angle_m + (1 - K1) * (angle1 + gyro_m * dt);
}

//二阶互补滤波 (经测试和kalman的效果非常一致)
static float K2 = 0.2;       // 对加速度计取值的权重
static float x1, x2, y1;
//static float dt = 0.001;     //注意：dt的取值为滤波器采样时间
float angle2 = 0;

void Erjielvbo(float angle_m, float gyro_m) //采集后计算的角度和角加速度
{
    x1 = (angle_m - angle2) * (1 - K2) * (1 - K2);
    y1 = y1 + x1 * dt;
    x2 = y1 + 2 * (1 - K2) * (angle_m - angle2) + gyro_m;
    angle2 = angle2 + x2 * dt;
}
