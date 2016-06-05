#include <math.h>
#include <stdio.h>
#include "app.h"
#include "tim.h"
#include "gpio.h"
#include "kalman.h"
#include "pid.h"
#include "i2c.h"
#include "mpu6050.h"
//#include "balancefilter.h"

/*
 * 平衡小车控制
 * 作者:刘帅
 * 2016-06-03
 */

#if defined(CANNON_V2)
const char* board_name = "CANNON V2";
#elif defined(CANNON_V1)
const char* board_name = "CANNON V1";
#endif

/******************for car******************/
uint16_t TIMER_CCR_MAX = 700;     //小车转速的最大值 0-999
static float pwm_f_coeff = 1.1f;  //前进时候的pwm系数修正
static float pwm_b_coeff = 1.2f;  //后退时候的pwm系数修正
static float balance_point= -1.5f;//平衡时的角度
static float set_speed   = 0.f;   //设置速度(实际上是角度)
//static float right_speed = 0.f;   //设置左轮速度
//static float left_speed  = 0.f;   //设置右轮速度
static float kp = 3.2f;           //比例项
static float ki = 0.05f;          //积分项
static float kd = 50.0f;          //微分项
static float dead_line = 0.5f;    //PID死区

//static float dt        = 0.004f;  //微分时间
//static float acc_coeff = 0.0037f; //加速度转换角度系数
static float atan_coeff  = 57.3f;   //arctan转换角度系数
static float gro_coeff = -0.0645f;  //角速度对应角度变化系数
static float gro_zero  = 24.05f;    //角速度为0时的校准值
extern float Angle, Gyro_y;         //kalman融合后得出的最优角度和角速度
//extern float angle1, angle2;      //一阶滤波和二阶滤波的结果

/*********************************
 *  小车控制函数 每4ms执行一次    *
**********************************/
void car_task(void* arg)
{
    run_after_delay(car_task, NULL, 4);
    int16_t ax, az;
    int16_t gy;
    
    float angle_a;
    float angle_g;
    //static float angle_g_last = 0;
    
    float pwm;
    
    uint32_t tmp;

    ax = 0;
    az = 0;
    gy = 0;
    ax = GetData(ACCEL_XOUT_H);
    az = GetData(ACCEL_ZOUT_H);
    gy = GetData(GYRO_YOUT_H);
#if 0
    printf("C1:%d\r\n", ax);
    printf("C2:%d\r\n", az);
#endif

#if 1
    /*
     *加速度计解算角度
     */
    //方法一(经测试 小于|40°|时 非常符合)：
    //angle_a = (float)ax*acc_coeff;
    //printf("C1:%f\r\n", angle_a);
    //方法二：
    angle_a = (float)atan((float)ax/az)*atan_coeff;
    printf("C1:%f\r\n", angle_a);
    
    /*
     *角速度计积分计算角度
     *角度 = 上次角度+角速度*积分时间
     */
    //angle_g = angle_g_last + ((float)gy - gro_zero)*dt*f*gro_coeff;
    //angle_g_last = angle_g;
    angle_g = ((float)gy - gro_zero)*gro_coeff;
    //printf("C2:%f\r\n", angle_g);
    
    /*
     *加速度和角速度融合计算角度
     */
    Kalman_Filter(angle_a, angle_g);
    printf("C3:%f\r\n", Angle);
    
    //Erjielvbo(angle_a, angle_g);
    //printf("C2:%f\r\n", angle2);
#endif

#if 1
    pwm = Angle;
    pwm = pid(pwm, balance_point+set_speed, kp, ki, kd, dead_line);
    
    if(pwm<0)
    {
        //后倾
        //此时是后倾
        car_to_b();
        tmp = (uint32_t)((-pwm)*30*pwm_b_coeff);
        set_b_pwm(tmp);
    }
    else
    {
        //此时是前倾
        car_to_f();
        tmp = (uint32_t)(pwm*30*pwm_f_coeff);
        set_f_pwm(tmp);
    }
#endif
    BSP_LED_Toggle(LED0);
}

void MyInit(void)
{
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    car_to_f();
    set_f_pwm(0);
    
    InitMPU6050();
    
    car_task((void *)0);
}

/*
 * 以下为关蓝牙和其他传感器的初始化部分
 */
#define UPDATE_INTERVAL 10
static void read_temperature(void* arg);
static void read_humidity(void* arg);
static void read_pressure(void* arg);
static void read_mag(void* arg);
static void read_acc(void* arg);
static void read_gyro(void* arg);
static uint8_t running;
void on_ready(void)
{
    uint8_t tx_power_level = 7;
    uint16_t adv_interval = 100;
    uint8_t bdAddr[6];
    char name[32];

    running = 0;

    BSP_LED_On(LED0);

    /* Initialize all configured peripherals */
    
    HCI_get_bdAddr(bdAddr);
    sprintf(name, "%s %01x%01x", board_name, bdAddr[0], bdAddr[1]);
    /*Config Adv Parameter And Ready to Adv*/
    ble_set_adv_param(name, bdAddr, tx_power_level, adv_interval);
    ble_device_start_advertising();
    
    BSP_LED_Off(LED0);
    
    MyInit();
}

//此函数已被main函数调用
void jsensor_app_set_sensors(void)
{
    jsensor_app_set_sensor(JSENSOR_TYPE_HUMITY_TEMP);
    jsensor_app_set_sensor(JSENSOR_TYPE_PRESSURE);
    //jsensor_app_set_sensor(JSENSOR_TYPE_MOTION_6AXIS);
    jsensor_app_set_sensor(JSENSOR_TYPE_MAGNET);
}

static void read_temperature(void* arg)
// sensor read temperature
{
    int16_t humidity;
    int16_t temperature;
    JSensor_HUM_TEMP_Typedef tdef;

    if (!running) return;

    tdef.humidity = &humidity;
    tdef.temperature = &temperature;

    if (JSENSOR_OK == jsensor_app_read_sensor(JSENSOR_TYPE_HUMITY_TEMP, (void *)&tdef)) {
        ble_device_send(0x00, 2, (uint8_t *)&temperature);
    }

    run_after_delay(read_humidity, NULL, UPDATE_INTERVAL);
}

static void read_humidity(void* arg)
// sensor read humidity
{
    int16_t humidity;
    int16_t temperature;
    JSensor_HUM_TEMP_Typedef tdef;

    if (!running) return;

    tdef.humidity = &humidity;
    tdef.temperature = &temperature;

    if (JSENSOR_OK == jsensor_app_read_sensor(JSENSOR_TYPE_HUMITY_TEMP, (void *)&tdef)) {
        ble_device_send(0x01, 2, (uint8_t *)&humidity);
    }

    run_after_delay(read_pressure, NULL, UPDATE_INTERVAL);
}

static void read_pressure(void* arg)
// sensor read pressure
{
    JSensor_Press_Typedef tdef;
    int32_t pressure;

    if (!running) return;

    tdef.pressure = &pressure;

    if (JSENSOR_OK == jsensor_app_read_sensor(JSENSOR_TYPE_PRESSURE, (void *)&tdef)) {
        ble_device_send(0x02, 3, (uint8_t *)&pressure);
    }

    run_after_delay(read_mag, NULL, UPDATE_INTERVAL);
}

static void read_mag(void* arg)
// sensor read magenetometer
{
    JSensor_MAG_Typedef tdef;
    int8_t  MAG[6];

    if (!running) return;

    tdef.MAG = MAG;

    if(JSENSOR_OK == jsensor_app_read_sensor(JSENSOR_TYPE_MAGNET, (void *)&tdef)) {
        ble_device_send(0x03, 6, (uint8_t*)MAG);
        //printf("%x,%x,%x,%x,%x,%x\n\r", MAG[0],MAG[1],MAG[2],MAG[3],MAG[4],MAG[5]);
    }

    run_after_delay(read_acc, NULL, UPDATE_INTERVAL);
}

static void read_acc(void* arg)
// sensor read accelerometer
{
    JSensor_AXIS_Typedef tdef;
    int8_t ACC[6], GRO[6];
    
    if (!running) return;

    tdef.ACC = ACC;
    tdef.GRO = GRO;

    if (JSENSOR_OK == jsensor_app_read_sensor(JSENSOR_TYPE_MOTION_6AXIS, (void *)&tdef)) {
        //ble_device_send(0x04, 6, (uint8_t*)ACC);
    }
    
    run_after_delay(read_gyro, NULL, UPDATE_INTERVAL);
}

static void read_gyro(void* arg)
// sensor read gyroscopic
{
    JSensor_AXIS_Typedef tdef;
    int8_t ACC[6], GRO[6];

    if (!running) return;

    tdef.ACC = ACC;
    tdef.GRO = GRO;

    if (JSENSOR_OK == jsensor_app_read_sensor(JSENSOR_TYPE_MOTION_6AXIS, (void *)&tdef)) {
        ble_device_send(0x05, 6, (uint8_t*)GRO);
    }

    run_after_delay(read_temperature, NULL, UPDATE_INTERVAL);
}

/* Device On Message */
void ble_device_on_message(uint8_t type, uint16_t length, uint8_t* value)
{
    float tmp = 0;
    if(type == 9)
    {
        //按键
    }
    else if(type == 1)
    {
        //预留
    }
    else if(type == 2)
    {
        //reset balance_point
        value[2] = '\0';                    //2位整数
        tmp = atoi((const char *)value);    //max = 20
        tmp -= 10;                          //-10  10
        set_speed = tmp;
    }
    else if(type == 3)
    {
        //reset kp
    }
    else if(type == 4)
    {
        //reset kd
    }
}

/* Device on connect */
void ble_device_on_connect(void)
{
    running = 1;

    BSP_LED_On(LED0);
    
    //run_after_delay(led_off, NULL, 150);
    //run_after_delay(read_temperature, NULL, UPDATE_INTERVAL);
}

/* Device on disconnect */
void ble_device_on_disconnect(uint8_t reason)
{
    running = 0;
    BSP_LED_Off(LED0);
    /* Make the device connectable again. */
    Ble_conn_state = BLE_CONNECTABLE;
    ble_device_start_advertising();
}
