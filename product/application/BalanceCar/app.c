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
 * ƽ��С������
 */

#if defined(CANNON_V2)
const char* board_name = "CANNON V2";
#elif defined(CANNON_V1)
const char* board_name = "CANNON V1";
#endif

/******************for car******************/
uint16_t TIMER_CCR_MAX = 700;     //С��ת�ٵ����ֵ 0-999
static float pwm_f_coeff = 1.1f;  //ǰ��ʱ���pwmϵ������
static float pwm_b_coeff = 1.2f;  //����ʱ���pwmϵ������
static float set_point   = -1.5f; //ƽ��ʱ�ĽǶ�
static float kp = 3.2f;           //������
static float ki = 0.05f;          //������
static float kd = 50.0f;          //΢����
static float dead_line = 0.5f;    //PID����

//static float dt        = 0.004f;  //΢��ʱ��
//static float acc_coeff = 0.0037f; //���ٶ�ת���Ƕ�ϵ��
static float gro_coeff = -0.0645f;  //���ٶȶ�Ӧ�Ƕȱ仯ϵ��
static float gro_zero  = 24.05f;    //���ٶ�Ϊ0ʱ��У׼ֵ
extern float Angle, Gyro_y;         //kalman�ںϺ�ó������ŽǶȺͽ��ٶ�
//extern float angle1, angle2;      //һ���˲��Ͷ����˲��Ľ��

/*********************************
 *  С�����ƺ��� ÿ4msִ��һ��    *
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
     *���ٶȼƽ���Ƕ�
     */
    //����һ(������ С��|40��|ʱ �ǳ�����)��
    //angle_a = (float)ax*acc_coeff;
    //printf("C1:%f\r\n", angle_a);
    //��������
    angle_a = (float)atan((float)ax/az)*57.3f;
    printf("C1:%f\r\n", angle_a);
    
    /*
     *���ٶȼƻ��ּ���Ƕ�
     *�Ƕ� = �ϴνǶ�+���ٶ�*����ʱ��
     */
    //angle_g = angle_g_last + ((float)gy - gro_zero)*dt*f*gro_coeff;
    //angle_g_last = angle_g;
    angle_g = ((float)gy - gro_zero)*gro_coeff;
    //printf("C2:%f\r\n", angle_g);
    
    /*
     *���ٶȺͽ��ٶ��ںϼ���Ƕ�
     */
    Kalman_Filter(angle_a, angle_g);
    printf("C3:%f\r\n", Angle);
    
    //Erjielvbo(angle_a, angle_g);
    //printf("C2:%f\r\n", angle2);
#endif

#if 1
    pwm = Angle;
    pwm = pid(pwm, set_point, kp, ki, kd, dead_line);
    
    if(pwm<0)
    {
        //����
        //��ʱ�Ǻ���
        car_to_b();
        tmp = (uint32_t)((-pwm)*30*pwm_b_coeff);
        set_b_pwm(tmp);
    }
    else
    {
        //��ʱ��ǰ��
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
 * ����Ϊ�������������������ĳ�ʼ������
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

//�˺����ѱ�main��������
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
volatile uint8_t pwm_is_on = 0;
volatile float  step_set  = 1;
void ble_device_on_message(uint8_t type, uint16_t length, uint8_t* value)
{
    float tmp = 0;
    if(type == 9)
    {
        /*LED Control*/
        if(*value == 0) {
            car_to_f();
            BSP_LED_On(LED0);
        }else if(*value == 1) {
            car_to_b();
            BSP_LED_Off(LED0);
        }else if(*value == 2) {
            //pwm off
            pwm_is_on = 0;

            if(++step_set == 4)
            {
                step_set = 1;
            }
        }else if(*value == 3) {
            //pwm on
            pwm_is_on = 1;
        }
    }
    else if(type == 1)
    {
        value[4] = '\0';
        TIM1->CCR1 = atoi((const char *)value) * 10;  //value is 0 ~ 100
        TIM2->CCR1 = TIM1->CCR1;
    }
    else if(type == 2)
    {
        //reset set_point
        value[4] = '\0';
        tmp = atoi((const char *)value)/50;
        tmp -= 500;
        tmp = tmp/50;
        set_point = tmp;
    }
    else if(type == 3)
    {
        //reset kp
        value[4] = '\0';
        tmp = atoi((const char *)value)/50;
        tmp -= 500;
        tmp = tmp/50;
        kp = tmp;
    }
    else if(type == 4)
    {
        //reset kd
        value[4] = '\0';
        tmp = atoi((const char *)value)/50;
        tmp -= 500;
        tmp = tmp/50;
        kd = tmp;
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
