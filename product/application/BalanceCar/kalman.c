
#include "kalman.h"

/*
 * kalmanƽ���˲���
 * ����:��˧
 * 2016-06-03
 * r���������˲����������ʵ�����ߵ�����̶ȣ�rԽСԽ�ӽ���
 * q�������˲��������ƽ���̶ȣ�qԽСԽƽ����
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
// �������˲�
//*********************************************************
//�ڳ���������Angle+=(Gyro - Q_bias) * dt����������ǻ��ֳ��ĽǶȣ�����Q_bias��������ƫ�
//��ʱ���������ǻ��������Angle�൱��ϵͳ�Ĺ���ֵ���õ�ϵͳ�Ĺ۲ⷽ�̣������ٶȼƼ��ĽǶ�Accel�൱��ϵͳ�еĲ���ֵ���õ�ϵͳ״̬���̡�
//������Q_angle��Q_gyro�ֱ��ʾϵͳ�Լ��ٶȼƼ������ǵ����ζȡ�����Pdot = A*P + P*A' + Q_angle������������Э�����΢�֣����ڽ���ǰ����ֵ�������Ի���������AΪ�ſ˱Ⱦ���
//������ϵͳԤ��Ƕȵ�Э�������P���������ֵAccel��Ԥ��ֵAngle������Angle_err��
//���㿨��������K_0,K_1��K_0�������Ź���ֵ��K_1���ڼ������Ź���ֵ��ƫ�����Э�������P��
//ͨ�������������������Ź���ֵAngle��Ԥ��ֵƫ��Q_bias����ʱ�õ����ŽǶ�ֵAngle�����ٶ�ֵ��
//Kalman�˲���20MHz�Ĵ���ʱ��Լ0.77ms��STC12C5A60S2��

//******����������************
static float Q_angle = 0.001;//�Ƕ��������Ŷ�
static float Q_gyro  = 0.005;//���ٶ��������Ŷ�
static float R_angle = 0.5;
static float dt  = 0.004;    //dtΪkalman�˲�������ʱ��;
static char  C_0 = 1;
static float Q_bias, Angle_err;
static float PCt_0, PCt_1, E;
static float K_0, K_1, t_0, t_1;
static float Pdot[4]  = {0, 0, 0, 0};
static float PP[2][2] = { { 1, 0 }, { 0, 1 } };
//�ⲿҪ�õĽǶȺͽ��ٶ�
float Angle;
float Gyro_y;

void Kalman_Filter(float Accel, float Gyro)
{
    Angle += (Gyro - Q_bias) * dt; //�������
    Angle_err = Accel - Angle;     //zk-�������

    // Pk-����������Э�����΢��
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] =  Q_gyro;

    // Pk-����������Э����΢�ֵĻ���=����������Э����
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

    // ����������Э����
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle    += K_0 * Angle_err;   //�������  (���ŽǶ�)
    Q_bias   += K_1 * Angle_err;   //�������
    Gyro_y   =  Gyro - Q_bias;     //���ֵ(�������)��΢��=���ٶ�  (���Ž��ٶ�)
}

