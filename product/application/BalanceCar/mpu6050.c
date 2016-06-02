//****************************************
// MPU6050
// MCU:STC15
// ��˧
// ���ڣ�2015/10/01
// ����: ��ʾ���ٶȼƺ������ǵ�10λԭʼ����
//****************************************

#include "mpu6050.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;

//****************************************
//��������
//****************************************	

//����IO
static void GPIO_config(void)
{
    HAL_I2C_MspInit(&hi2c1);
    MX_I2C1_Init();
}

//**************************************
//��I2C�豸д��һ���ֽ�����
//**************************************
void Single_WriteI2C(uint8_t REG_Address,uint8_t REG_data)
{
    /*
    I2C_Start();                  //��ʼ�ź�
    I2C_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    I2C_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
    I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    I2C_Stop();                   //����ֹͣ�ź�
    */
    uint8_t data[2] = {REG_Address, REG_data};
    HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, data, 2, 0x00ffffff);
}
//**************************************
//��I2C�豸��ȡһ���ֽ�����
//**************************************
uint8_t Single_ReadI2C(uint8_t REG_Address)
{
    uint8_t REG_data;
    /*
    I2C_Start();                   //��ʼ�ź�
    I2C_SendByte(SlaveAddress);    //�����豸��ַ+д�ź�
    I2C_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
    I2C_Start();                   //��ʼ�ź�
    I2C_SendByte(SlaveAddress+1);  //�����豸��ַ+���ź�
    REG_data=I2C_RecvByte();       //�����Ĵ�������
    I2C_SendACK(1);                //����Ӧ���ź�
    I2C_Stop();                    //ֹͣ�ź�
    */
    HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, &REG_Address, 1, 0x00ffffff);
    HAL_I2C_Master_Receive(&hi2c1, SlaveAddress+1, &REG_data, 1, 0x00ffffff);
    return REG_data;
}
//**************************************
//��ʼ��MPU6050
//**************************************
void InitMPU6050()
{
    GPIO_config();

    Single_WriteI2C(PWR_MGMT_1, 0x00);	//�������״̬
    Single_WriteI2C(SMPLRT_DIV, 0x07);
    Single_WriteI2C(CONFIG, 0x06);
    Single_WriteI2C(GYRO_CONFIG, 0x18);
    Single_WriteI2C(ACCEL_CONFIG, 0x01);
}
//**************************************
//�ϳ�����
//**************************************
int16_t GetData(uint8_t REG_Address)
{
    uint8_t L;
    int16_t H;
    H=Single_ReadI2C(REG_Address);
    L=Single_ReadI2C(REG_Address+1);
    //�ϳ�����
    H <<= 8;
    H |= (L&0x00FF);
    return H;
}
