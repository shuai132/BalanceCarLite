#include "balancefilter.h"

//һ�׻����˲� (���Է���Ч����̫����)
static float K1 = 0.05;       // �Լ��ٶȼ�ȡֵ��Ȩ��
float dt = 0.004;      //ע�⣺dt��ȡֵΪ�˲�������ʱ��
float angle1;

void Yijielvbo(float angle_m, float gyro_m)//�ɼ������ĽǶȺͽǼ��ٶ�
{
    angle1 = K1 * angle_m + (1 - K1) * (angle1 + gyro_m * dt);
}

//���׻����˲� (�����Ժ�kalman��Ч���ǳ�һ��)
static float K2 = 0.2;       // �Լ��ٶȼ�ȡֵ��Ȩ��
static float x1, x2, y1;
//static float dt = 0.001;     //ע�⣺dt��ȡֵΪ�˲�������ʱ��
float angle2 = 0;

void Erjielvbo(float angle_m, float gyro_m) //�ɼ������ĽǶȺͽǼ��ٶ�
{
    x1 = (angle_m - angle2) * (1 - K2) * (1 - K2);
    y1 = y1 + x1 * dt;
    x2 = y1 + 2 * (1 - K2) * (angle_m - angle2) + gyro_m;
    angle2 = angle2 + x2 * dt;
}
