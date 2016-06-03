
#include "pid.h"
#include <math.h>

/*
 * PID �㷨
 * ����:��˧
 * 2016-06-03
 */

float pid(float input, float setpoint, float kp, float ki, float kd, float dead_line)
{
    float output;
    float error = input - setpoint;
    static float error_last = 0;
    static float error_sum  = 0;

    output = 0;
    if (error < 30.0f || error > -30.0f)
    {
        //ƫ����30������
        if (((float)(fabs(error))) > dead_line)
        {
            //��������
            error_sum += error;
            output = error * kp + ki * error_sum + kd * (error - error_last);
            error_last = error;
        }
    }

    return output;
}
