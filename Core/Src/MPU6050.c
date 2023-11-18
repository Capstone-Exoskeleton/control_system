#include "i2c.h"
#include "MPU6050.h"
#include "MPU6050_reg.h"

#include "stdio.h"

#define MPU6050_ADDRESS 0xD0

// MPU6050 Config
#define MPU6050_ACC_SCALE 0x1000
#define MPU6050_GYRO_SCALE 0x83

// Measured Offset
#define GX_OFFSET -1500
#define GY_OFFSET 68
#define GZ_OFFSET -3

/**
  ******************************************************************
  * @brief   Write to MPU6050
  * @param   [in]RegAddress Resigter Address in MPU6050
  * @param   [in]Data
  * @author  Yuying
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
    HAL_I2C_Master_Transmitx(NULL, MPU6050_ADDRESS, RegAddress, &Data, 1, 0xffff);
}

/**
  ******************************************************************
  * @brief   Read from MPU6050
  * @param   [in]RegAddress Resigter Address in MPU6050
  * @return  uint8_t Data
  * @author  Yuying
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    
    if (HAL_I2C_Master_Transmitx(NULL, MPU6050_ADDRESS, RegAddress, &Data, 0, 0xffff) == HAL_OK) // point to register
			HAL_I2C_Master_Receivex(NULL, MPU6050_ADDRESS|0x01/*for read*/, &Data, 1, 0xffff);
		else
			return 0;
    
    return Data;
    
    
}

/**
  ******************************************************************
  * @brief   Initialize MPU6050
  * @author  Yuying
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
void MPU6050_Init()
{
    Soft_I2C_Init();
		while (MPU6050_ReadReg(MPU6050_PWR_MGMT_1) != 0x01)
		{
			HAL_Delay(2);
			MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01); //wake，use imu clock
		}
		while (MPU6050_ReadReg(MPU6050_PWR_MGMT_2) != 0x00)
		{
			HAL_Delay(2);
			MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00); //wake all measurement units
		}
		while (MPU6050_ReadReg(MPU6050_SMPLRT_DIV) != 0x07)
		{
			HAL_Delay(2);
			MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x07); //Sample Rate Devider 125Hz
		}
		while (MPU6050_ReadReg(MPU6050_CONFIG) != 0x06)
		{
			HAL_Delay(2);
			MPU6050_WriteReg(MPU6050_CONFIG, 0x06);  //Filter Bankwidth 5Hz
		}
		while (MPU6050_ReadReg(MPU6050_GYRO_CONFIG) != 0x18)
		{
			HAL_Delay(2);
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); //2000 degree
		}
		while (MPU6050_ReadReg(MPU6050_ACCEL_CONFIG) != 0x0)
		{
			HAL_Delay(2);
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x0); //2G
		}
}

/**
  ******************************************************************
  * @brief   Get MPU6050 ID from who_am_i reg
  * @author  Yuying
  * @return  uint8_t ID
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
uint8_t MPU6050_GetID()
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
  ******************************************************************
  * @brief   Read all measured data from MPU6050
  * @param   [in]AccX Accelerameter X
  * @param   [in]AccY Accelerameter Y
  * @param   [in]AccZ Accelerameter Z
  * @param   [in]GyroX Gyroscopte X
  * @param   [in]GyroY Gyroscopte Y
  * @param   [in]GyroZ Gyroscopte Z
  * @author  Yuying
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
void MPU6050_GetData(int16_t* AccX,int16_t* AccY,int16_t* AccZ,int16_t* GyroX,int16_t* GyroY,int16_t* GyroZ)
{
    uint8_t DataH,DataL;
    
    DataH=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX=(int16_t)((DataH<<8)|DataL);
    
    DataH=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY=(int16_t)((DataH<<8)|DataL);
    
    DataH=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ=(int16_t)((DataH<<8)|DataL);
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (int16_t)((DataH<<8)|DataL)+ GX_OFFSET;
	
    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (int16_t)((DataH<<8)|DataL)+ GY_OFFSET;
	
    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (int16_t)((DataH<<8)|DataL)+ GZ_OFFSET;

}
