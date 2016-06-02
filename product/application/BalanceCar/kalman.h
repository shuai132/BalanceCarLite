#ifndef _KALMAN_H
#define _KALMAN_H

float kalman(float y);
void Kalman_Filter(float angle_m,float gyro_m);
void Yijielvbo(float angle_m, float gyro_m);
void Erjielvbo(float angle_m, float gyro_m);

#endif



/*
其基本思想就是先不考虑输入信号和观测噪声的影响，得到状态变量和输出信号的估计值，
再用输出信号的估计误差加权后校正状态变量的估计值，使状态变量估计误差的均方差最小。

*/

