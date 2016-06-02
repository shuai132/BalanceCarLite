//****************************************
// MPU6050
// MCU:STC15
// 刘帅
// 日期：2015/10/01
// 功能: 显示加速度计和陀螺仪的10位原始数据
//****************************************

#include "mpu6050.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;

//****************************************
//函数声明
//****************************************	

//配置IO
static void GPIO_config(void)
{
    HAL_I2C_MspInit(&hi2c1);
    MX_I2C1_Init();
}

//**************************************
//向I2C设备写入一个字节数据
//**************************************
void Single_WriteI2C(uint8_t REG_Address,uint8_t REG_data)
{
    /*
    I2C_Start();                  //起始信号
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号
    I2C_SendByte(REG_Address);    //内部寄存器地址，
    I2C_SendByte(REG_data);       //内部寄存器数据，
    I2C_Stop();                   //发送停止信号
    */
    uint8_t data[2] = {REG_Address, REG_data};
    HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, data, 2, 0x00ffffff);
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
uint8_t Single_ReadI2C(uint8_t REG_Address)
{
    uint8_t REG_data;
    /*
    I2C_Start();                   //起始信号
    I2C_SendByte(SlaveAddress);    //发送设备地址+写信号
    I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始
    I2C_Start();                   //起始信号
    I2C_SendByte(SlaveAddress+1);  //发送设备地址+读信号
    REG_data=I2C_RecvByte();       //读出寄存器数据
    I2C_SendACK(1);                //接收应答信号
    I2C_Stop();                    //停止信号
    */
    HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, &REG_Address, 1, 0x00ffffff);
    HAL_I2C_Master_Receive(&hi2c1, SlaveAddress+1, &REG_data, 1, 0x00ffffff);
    return REG_data;
}
//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050()
{
    GPIO_config();

    Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
    Single_WriteI2C(SMPLRT_DIV, 0x07);
    Single_WriteI2C(CONFIG, 0x06);
    Single_WriteI2C(GYRO_CONFIG, 0x18);
    Single_WriteI2C(ACCEL_CONFIG, 0x01);
}
//**************************************
//合成数据
//**************************************
int16_t GetData(uint8_t REG_Address)
{
    uint8_t L;
    int16_t H;
    H=Single_ReadI2C(REG_Address);
    L=Single_ReadI2C(REG_Address+1);
    //合成数据
    H <<= 8;
    H |= (L&0x00FF);
    return H;
}
