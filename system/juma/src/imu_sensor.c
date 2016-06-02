#include "imu_sensor.h"
#include "app.h"

#if NO_PRINTF
#define printf(...)
#endif

static sensor_selsection_t sensor_selection;
static imu_sensor_data_sensitivity_t sensor_data_sensitivity;
static sensor_data_read_param_t sensor_data_param;

__weak void on_imu_sensor_data(imu_sensor_data_t* data) {};

static imu_status_t lsm6ds3_fifo_sensor_enable(void);
static imu_status_t imu_sensor_config_acc(uint8_t data_rate, uint8_t scale);
static imu_status_t imu_sensor_config_gyro(uint8_t data_rate, uint8_t scale);
static imu_status_t imu_sensor_acc_output_status_config(uint8_t status);
static imu_status_t imu_sensor_gyro_output_status_config(uint8_t status);
#ifdef LSM6DS3_THRESHOLD
static imu_status_t imu_sensor_fifo_threshold_interrupt(void);
static imu_status_t imu_sensor_fifo_threshold_level(uint16_t fifo_level);
#endif
#ifdef LSM6DS3_SOFT_RESET
static imu_status_t imu_sensor_lsm6ds3_soft_reset(void);
#endif
#ifdef LSM303AGR_SOFT_RESET
static imu_status_t imu_sensor_lsm303agr_soft_rest(void);
#endif
static imu_status_t imu_sensor_acc_get_sensitivity( float *pfData );
static imu_status_t  imu_sensor_gyro_get_sensitivity( float *pfData );
#ifdef LSM6DS3_CLEAR_FIFO
static imu_status_t imu_sensor_clear_fifo(void);
#endif
static imu_status_t imu_sensor_fifo_data_number(uint16_t* number);
static imu_status_t imu_sensor_read_sensor_rate_config(uint8_t number);
static void imu_sensor_read_fifo_delay(void);

/*reset sensors*/
imu_status_t imu_sensor_reset(void)
{

    if(LSM6DS3_IO_Init() != imu_status_ok)
    {
        printf("lsm6ds3 io init error\n");
        return imu_status_fail;
    }
    /* Configure interrupt lines */
    LSM6DS3_IO_ITConfig();
    printf("IT IO Config\n");

    if(lsm6ds3_fifo_sensor_enable() != imu_status_ok)
    {
        printf("sensor fifoenable error\n");
        return imu_status_fail;
    }
    printf("sensor enable\n");
    return imu_status_ok;

}

/*active sensor*/
imu_status_t imu_sensor_select_features(sensor_selsection_t features)
{

    sensor_selection = features;

    printf("sensor features : %x\n", sensor_selection);

    return imu_status_ok;
}

/*set data rate*/
imu_status_t imu_sensor_set_data_rate(uint32_t* p_data_rate, uint8_t mode)
{

    uint8_t tmp1 = 0x00;
    uint8_t new_odr = 0x00;
    /*lsm6ds3*/
    sensor_data_param.sample_rate = * p_data_rate;
    printf("fifo odr:%x\n",* p_data_rate);
    {
        if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL5, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }

        /* FIFO ODR selection */
        switch(* p_data_rate) {
        case 0:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_NA;
        case 10:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_10HZ;
            break;
        case 25:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_25HZ;
            break;
        case 50:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_50HZ;
            break;
        case 100:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_100HZ;
            break;
        case 200:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_200HZ;
            break;
        case 400:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_400HZ;
            break;
        case 800:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_800HZ;
            break;
        case 1600:
            tmp1 |= LSM6DS3_XG_FIFO_ODR_1600HZ;
            break;

        default:
            break;
        }

        /* FIFO mode selection */
        tmp1 &= ~(LSM6DS3_XG_FIFO_MODE_MASK);
        tmp1 |= LSM6DS3_XG_FIFO_MODE_FIFO;

        if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL5, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }

    }
    /*acc and gyro odr */
    {
        new_odr = ( * p_data_rate <= 0    )     ? LSM6DS3_XL_ODR_PD          /* Power Down */
                  : ( * p_data_rate <= 13   )  ? LSM6DS3_XL_ODR_13HZ
                  : ( * p_data_rate <= 26   )  ? LSM6DS3_XL_ODR_26HZ
                  : ( * p_data_rate <= 52   )  ? LSM6DS3_XL_ODR_52HZ
                  : ( * p_data_rate <= 104  )  ? LSM6DS3_XL_ODR_104HZ
                  : ( * p_data_rate <= 208  )  ? LSM6DS3_XL_ODR_208HZ
                  : ( * p_data_rate <= 416  )  ? LSM6DS3_XL_ODR_416HZ
                  : ( * p_data_rate <= 833  )  ? LSM6DS3_XL_ODR_833HZ
                  :                          LSM6DS3_XL_ODR_1660HZ;
        if(sensor_selection & ACC_ENABLE) {

            if(imu_sensor_config_acc(new_odr, LSM6DS3_XL_FS_2G) != imu_status_ok) {
                return imu_status_fail;

            }
        }
        if(sensor_selection & GYRO_ENABLE) {

            if(imu_sensor_config_gyro(new_odr, LSM6DS3_G_FS_125_ENABLE) != imu_status_ok) {
                return imu_status_fail;
            }
        }

        printf("acc and gyro odr set over\n");
    }
    /*lsm303agr*/
    {
        if(LSM303AGR_MAG_W_ODR(LSM303AGR_MAG_ODR_100Hz) != imu_status_ok)
        {
            return imu_status_fail;
        }
        printf("mag odr set over\n");
    }


    return imu_status_ok;
}

/*start get sensor data*/
imu_status_t imu_sensor_start(void)
{

    /*lsm6ds3*/
    {
        if(sensor_selection & ACC_ENABLE) {
            if(imu_sensor_acc_output_status_config(OUTPUT_ENABLE) != imu_status_ok)
            {
                return imu_status_fail;
            }
            printf("acc output enable\n");
        }
        if(sensor_selection & GYRO_ENABLE) {
            if(imu_sensor_gyro_output_status_config(OUTPUT_ENABLE) != imu_status_ok)
            {
                return imu_status_fail;
            }
            printf("gyro output enable\n");
        }

    }
    /*lsm303agr odr*/
    {
        if(LSM303AGR_MAG_W_MD(LSM303AGR_MAG_MD_CONTINUOS_MODE) != imu_status_ok)
        {
            return imu_status_fail;
        }
        printf("mag output enable\n");
    }
    imu_sensor_read_sensor_rate_config(60);
    imu_sensor_read_data_from_fifo(NULL);

    return imu_status_ok;
}

/*stop get sensor data*/
imu_status_t imu_sensor_stop(void)
{
    /*lsm6ds3*/
    {

        if(sensor_selection & ACC_ENABLE) {
            if(imu_sensor_acc_output_status_config(OUTPUT_DISABLE) != imu_status_ok)
            {
                return imu_status_fail;
            }
            printf("acc output disable\n");
        }

        if(sensor_selection & GYRO_ENABLE) {
            if(imu_sensor_gyro_output_status_config(OUTPUT_DISABLE) != imu_status_ok)
            {
                return imu_status_fail;
            }
            printf("gyro output disable\n");
        }

    }
    /*lsm303agr odr*/
    {
        if(LSM303AGR_MAG_W_MD(LSM303AGR_MAG_MD_IDLE1_MODE) != imu_status_ok)
        {
            return imu_status_fail;
        }
        printf("mag output disable\n");
    }

    return imu_status_ok;
}


/*fifo config*/
static imu_status_t lsm6ds3_fifo_sensor_enable(void)
{
    uint8_t tmp1 = 0x00;
    /*fifo decimation setting*/
    {
        if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL3, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }
        /*set */
        tmp1 &= ~(0x3F);
        tmp1 |= 0x09;
        if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL3, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }

    }
    /*lsm6ds3*/
    {

        if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL3_C, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }
        /* Enable register address automatically incremented during a multiple byte
           access with a serial interface (I2C or SPI) */
        tmp1 &= ~(LSM6DS3_XG_IF_INC_MASK);
        tmp1 |= LSM6DS3_XG_IF_INC;

        if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL3_C, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }

    }

    return imu_status_ok;

}

/*active acc*/
static imu_status_t imu_sensor_config_acc(uint8_t data_rate, uint8_t scale)
{
    uint8_t tempReg = 0x00;

    /*acc rate and scale*/
    if(LSM6DS3_IO_Read( &tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1 ) != imu_status_ok)
    {
        return imu_status_fail;
    }

    tempReg &= ~(LSM6DS3_XL_ODR_MASK);
    tempReg |= data_rate;
    /* Full scale selection */
    tempReg &= ~(LSM6DS3_XL_FS_MASK);
    tempReg |= scale;

    if(LSM6DS3_IO_Write(&tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    /*get sensitivity*/
    if(imu_sensor_acc_get_sensitivity( &sensor_data_sensitivity.acc_sensitivity ) != imu_status_ok)
    {
        return imu_status_fail;
    }
    printf("acc_sensitivity : %f\n", sensor_data_sensitivity.acc_sensitivity);
    return imu_status_ok;
}

/*active gyro*/
static imu_status_t imu_sensor_config_gyro(uint8_t data_rate, uint8_t scale)
{
    uint8_t tempReg = 0x00;

    /*gyro rate and scale*/
    if(LSM6DS3_IO_Read( &tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL2_G, 1 ) != imu_status_ok)
    {
        return imu_status_fail;
    }

    tempReg &= ~(LSM6DS3_G_ODR_MASK);
    tempReg |= data_rate;
    /* Full scale selection */
    tempReg &= ~(LSM6DS3_G_FS_125_MASK);
    tempReg |= scale;

    if(LSM6DS3_IO_Write(&tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL2_G, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    /*get sensitivity*/
    if(imu_sensor_gyro_get_sensitivity( &sensor_data_sensitivity.gyro_sensitivity ) != imu_status_ok)
    {
        return imu_status_fail;
    }

    printf("gyro_sensitivity : %f\n", sensor_data_sensitivity.gyro_sensitivity);

    return imu_status_ok;
}

/*set acc output state*/
static imu_status_t imu_sensor_acc_output_status_config(uint8_t status)
{
    uint8_t tmp1 = 0x00;
    uint8_t eX = 0x00;
    uint8_t eY = 0x00;
    uint8_t eZ = 0x00;
    if(status == OUTPUT_ENABLE) {

        eX = LSM6DS3_XL_XEN_ENABLE;
        eY = LSM6DS3_XL_YEN_ENABLE;
        eZ = LSM6DS3_XL_ZEN_ENABLE;
    }

    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL9_XL, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    /* Enable X axis selection */
    tmp1 &= ~(LSM6DS3_XL_XEN_MASK);
    tmp1 |= eX;

    /* Enable Y axis selection */
    tmp1 &= ~(LSM6DS3_XL_YEN_MASK);
    tmp1 |= eY;

    /* Enable Z axis selection */
    tmp1 &= ~(LSM6DS3_XL_ZEN_MASK);
    tmp1 |= eZ;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL9_XL, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    return imu_status_ok;
}

/*set gyro output state*/
static imu_status_t imu_sensor_gyro_output_status_config(uint8_t status)
{
    uint8_t tmp1 = 0x00;
    uint8_t eX = 0x00;
    uint8_t eY = 0x00;
    uint8_t eZ = 0x00;

    if(status == OUTPUT_ENABLE) {

        eX = LSM6DS3_G_XEN_ENABLE;
        eY = LSM6DS3_G_YEN_ENABLE;
        eZ = LSM6DS3_G_ZEN_ENABLE;
    }

    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL10_C, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    /* Enable X axis selection */
    tmp1 &= ~(LSM6DS3_G_XEN_MASK);
    tmp1 |= eX;

    /* Enable Y axis selection */
    tmp1 &= ~(LSM6DS3_G_YEN_MASK);
    tmp1 |= eY;

    /* Enable Z axis selection */
    tmp1 &= ~(LSM6DS3_G_ZEN_MASK);
    tmp1 |= eZ;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL10_C, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    return imu_status_ok;
}
/*config read fifo group number*/
static imu_status_t imu_sensor_read_sensor_rate_config(uint8_t number)
{
    sensor_data_param.group_number = number * 2;

    return imu_status_ok;
}

static void imu_sensor_read_fifo_delay(void)
{
    uint16_t fifo_remain_number;
    uint16_t  remain_group;
 
    if( imu_sensor_fifo_data_number(&fifo_remain_number) != imu_status_ok)
    {
        return;
    }
    printf("fifo_remain_number :%d\n", fifo_remain_number);
    remain_group = fifo_remain_number / 6;
    if(remain_group > sensor_data_param.group_number){
        sensor_data_param.delay_time = 0;
    }else{
        sensor_data_param.delay_time = ((sensor_data_param.group_number - remain_group) / 2) * (1000 / sensor_data_param.sample_rate);
    }
}

/*fifo read*/
void imu_sensor_read_data_from_fifo(void* arg)
{
    int16_t pData[3] = {0};
    uint8_t tempReg[2] = {0, 0}, number;
    sensor_data_type_t flag;
    imu_sensor_data_t sensor_data = {0.0,0.0,0.0};

    flag = TYPE_GYRO_DATA;
    /*mag data*/
    if(LSM303AGR_MAG_Get_Magnetic(sensor_data.mag) != imu_status_ok)
    {
        printf("read sensor error\n");
        return ;
    }

    number = sensor_data_param.group_number;
    while(number > 0) {

        if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_DATA_OUT_L, 2) != imu_status_ok)
        {
            printf("read sensor error\n");
            return ;
        }
        pData[0] = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

        if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_DATA_OUT_L, 2) != imu_status_ok)
        {
            printf("read sensor error\n");
            return ;
        }

        pData[1] = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

        if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_DATA_OUT_L, 2) != imu_status_ok)
        {
            printf("read sensor error\n");
            return ;
        }

        pData[2] = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

        if(flag == TYPE_GYRO_DATA) {
            /*gyro data*/
            sensor_data.gyro[0] = (float)((pData[0] * sensor_data_sensitivity.gyro_sensitivity)/1000);
            sensor_data.gyro[1] = (float)((pData[1] * sensor_data_sensitivity.gyro_sensitivity)/1000);
            sensor_data.gyro[2] = (float)((pData[2] * sensor_data_sensitivity.gyro_sensitivity)/1000);
            flag = TYPE_ACC_DATA;
        } else if (flag == TYPE_ACC_DATA) {

            /*acc data*/
            sensor_data.acc[0] = (float)(pData[0] * sensor_data_sensitivity.acc_sensitivity);
            sensor_data.acc[1] = (float)(pData[1] * sensor_data_sensitivity.acc_sensitivity);
            sensor_data.acc[2] = (float)(pData[2] * sensor_data_sensitivity.acc_sensitivity);
            on_imu_sensor_data(&sensor_data);
            flag = TYPE_GYRO_DATA;
        }

        number--;
    }
    
    imu_sensor_read_fifo_delay();
    run_after_delay(imu_sensor_read_data_from_fifo, NULL, sensor_data_param.delay_time);
}

#ifdef LSM6DS3_THRESHOLD
/*fifo threshold interrupt*/
static imu_status_t imu_sensor_fifo_threshold_interrupt(void)
{

    uint8_t tmp1;

    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_INT1_CTRL, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }
    /*enable fifo threshold interrupt*/
    tmp1 &= ~(LSM6DS3_XG_FIFO_INT_THRESHOLD_MASK);
    tmp1 |= LSM6DS3_XG_FIFO_INT_THRESHOLD_MASK;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_INT1_CTRL, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    return imu_status_ok;

}
#endif 

#ifdef LSM6DS3_THRESHOLD
/*fifo threshold level setting*/
static imu_status_t imu_sensor_fifo_threshold_level(uint16_t fifo_level)
{
    uint8_t tmp1;
    {
        if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL1, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }
        /*set water mark low 8bit*/
        tmp1 &= ~(0xFF);
        tmp1 |= (fifo_level & 0xFF);
        if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL1, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }

    }

    {
        if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL2, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }
        /*set water mark high 4bit*/
        tmp1 &= ~(LSM6DS3_XG_FIFO_THRESHOLD_MASK);
        tmp1 |= (fifo_level >> 8) & 0x0F;
        if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL2, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }

    }

    {
        if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL3, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }
        /*set */
        tmp1 &= ~(0x3F);
        tmp1 |= 0x09;
        if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL3, 1) != imu_status_ok)
        {
            return imu_status_fail;
        }

    }

    return imu_status_ok;

}
#endif

#ifdef LSM6DS3_SOFT_RESET
/*lsm6ds3 soft reset*/
static imu_status_t imu_sensor_lsm6ds3_soft_reset(void)
{
    uint8_t tmp1;

    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL3_C, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    /*soft reset*/
    tmp1 &= ~(0x01);
    tmp1 |= 0x01;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL3_C, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }
    HAL_Delay(1);

    return imu_status_ok;

}
#endif

#ifdef LSM303AGR_SOFT_RESET
/*lsm303agr soft reset*/
static imu_status_t imu_sensor_lsm303agr_soft_rest(void)
{
    uint8_t tmp1;

    if( LSM303AGR_MAG_ReadReg(LSM303AGR_MAG_CFG_REG_A, &tmp1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    tmp1 &= ~(0x20);
    tmp1 |= (0x20);

    if( LSM303AGR_MAG_WriteReg(LSM303AGR_MAG_CFG_REG_A, tmp1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    HAL_Delay(1);
    return imu_status_ok;
}
#endif

static imu_status_t imu_sensor_acc_get_sensitivity( float *pfData )
{
    /*Here we have to add the check if the parameters are valid*/

    uint8_t tempReg = 0x00;


    if(LSM6DS3_IO_Read( &tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1 ) != imu_status_ok)
    {
        return imu_status_fail;
    }

    tempReg &= LSM6DS3_XL_FS_MASK;

    switch( tempReg )
    {
    case LSM6DS3_XL_FS_2G:
        *pfData = 0.061f;
        break;
    case LSM6DS3_XL_FS_4G:
        *pfData = 0.122f;
        break;
    case LSM6DS3_XL_FS_8G:
        *pfData = 0.244f;
        break;
    case LSM6DS3_XL_FS_16G:
        *pfData = 0.488f;
        break;
    default:
        break;
    }

    return imu_status_ok;
}

#ifdef LSM6DS3_CLEAR_FIFO
static imu_status_t imu_sensor_clear_fifo(void)
{
    uint8_t tempReg[2] = {0, 0};
    uint32_t p_data_rate = 10;
    uint16_t fifo_data_number = 0;

    if(imu_sensor_fifo_data_number(&fifo_data_number) != imu_status_ok) {
        return imu_status_fail;
    }

    imu_sensor_set_data_rate(&p_data_rate, LSM6DS3_XG_FIFO_MODE_BYPASS);
    printf("set data rate to 0x00\n");
    uint16_t i;
    for(i = 0; i < fifo_data_number; i++) {
        if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_DATA_OUT_L, 2) != imu_status_ok)
        {
            printf("read sensor error\n");
            return imu_status_fail;
        }
    }
    printf("clear fifo over \n");
    return imu_status_ok;

}
#endif

static imu_status_t imu_sensor_fifo_data_number(uint16_t* number)
{
    uint8_t fifo_1_number = 0,fifo_2_number = 0;

    if(LSM6DS3_IO_Read(&fifo_1_number, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_STATUS1, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    if(LSM6DS3_IO_Read(&fifo_2_number, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_STATUS2, 1) != imu_status_ok)
    {
        return imu_status_fail;
    }

    *number |= (fifo_2_number & 0x0F);
    *number = (*number << 8) | fifo_1_number;

    return imu_status_ok;
}

static imu_status_t  imu_sensor_gyro_get_sensitivity( float *pfData )
{
    /*Here we have to add the check if the parameters are valid*/

    uint8_t tempReg = 0x00;

    if(LSM6DS3_IO_Read( &tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL2_G, 1 ) != imu_status_ok)
    {
        return imu_status_fail;
    }

    tempReg &= LSM6DS3_G_FS_125_MASK;

    if(tempReg == LSM6DS3_G_FS_125_ENABLE)
    {
        *pfData = 4.375f;
    }
    else
    {
        if(LSM6DS3_IO_Read( &tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL2_G, 1 ) != imu_status_ok)
        {
            return imu_status_fail;
        }

        tempReg &= LSM6DS3_G_FS_MASK;

        switch( tempReg )
        {
        case LSM6DS3_G_FS_245:
            *pfData = 8.75f;
            break;
        case LSM6DS3_G_FS_500:
            *pfData = 17.50f;
            break;
        case LSM6DS3_G_FS_1000:
            *pfData = 35.0f;
            break;
        case LSM6DS3_G_FS_2000:
            *pfData = 70.0f;
            break;
        default:
            break;
        }
    }

    return imu_status_ok;
}

