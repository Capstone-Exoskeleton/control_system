#include "i2c.h"
#include "kalman.h"

#include "math.h"
#include "stdio.h"

#define MPU6050_ADDRESS 0xD0

// MPU6050 Config
#define MPU6050_ACC_SCALE 0x1000
#define MPU6050_GYRO_SCALE 0x83

// Measured Offset
#define GX_OFFSET 190
#define GY_OFFSET 410
#define GZ_OFFSET 180

#define RAD_TO_DEG 57.295779513082320876798154814105

uint32_t timer;
MPU6050_t data;
MPU6050_t* sensor = &data;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.02f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};


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
			MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01); //wake,use imu clock
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
		while (MPU6050_ReadReg(MPU6050_GYRO_CONFIG) != 0x0)
		{
			HAL_Delay(2);
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x0); //2000 degree
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

float Get_kalman_angle()
{
	MPU6050_Read_All();
	return sensor->KalmanAngleY;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void MPU6050_Read_All() 
{
    uint8_t DataH,DataL;

		//x
    DataH=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    sensor->Accel_X_RAW=(int16_t)((DataH<<8)|DataL);

		//y
    DataH=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    sensor->Accel_Y_RAW=(int16_t)((DataH<<8)|DataL);
	
		//z
    DataH=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    sensor->Accel_Z_RAW= (int16_t)((DataH<<8)|DataL);

    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    sensor->Gyro_X_RAW = (int16_t)((DataH<<8)|DataL)+ GX_OFFSET;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    sensor->Gyro_Y_RAW = (int16_t)((DataH<<8)|DataL)+ GY_OFFSET;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    sensor->Gyro_Z_RAW = (int16_t)((DataH<<8)|DataL)+ GZ_OFFSET;

    sensor->Ax = sensor->Accel_X_RAW / 16384.0;
    sensor->Ay = sensor->Accel_Y_RAW / 16384.0;
    sensor->Az = sensor->Accel_Z_RAW / 16384.0f;
    sensor->Gx = sensor->Gyro_X_RAW / 131.0;
    sensor->Gy = sensor->Gyro_Y_RAW / 131.0;
    sensor->Gz = sensor->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
		/*
    double roll;
    double roll_sqrt = sqrt(
            sensor->Accel_X_RAW * sensor->Accel_X_RAW + sensor->Accel_Y_RAW * sensor->Accel_Y_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan((-1.0f)*sensor->Accel_Z_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
		*/
    double pitch = atan2(sensor->Accel_Y_RAW, (-1)*sensor->Accel_X_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && sensor->KalmanAngleY > 90) || (pitch > 90 && sensor->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        sensor->KalmanAngleY = pitch;
    } else {
        sensor->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, sensor->Gz, dt);
    }
		/*
    if (fabs(sensor->KalmanAngleY) > 90)
        sensor->Gx = -sensor->Gx;
    sensor->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, sensor->Gy, dt);
		*/
		//printf("%.2f %.2f,%.2f %.2f %.2f %d %d %d\n\r", sensor->KalmanAngleX, sensor->KalmanAngleY, sensor->Ax, sensor->Ay, sensor->Az, sensor->Gyro_X_RAW, sensor->Gyro_Y_RAW, sensor->Gyro_Z_RAW);
		printf("0/%.2f/0\r\n",sensor->KalmanAngleY);
}

