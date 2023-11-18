#ifndef __MPU6050_H__
#define __MPU6050_H__


void MPU6050_Init();
uint8_t MPU6050_GetID();
void MPU6050_GetData(int16_t* AccX,int16_t* AccY,int16_t* AccZ,int16_t* GyroX,int16_t* GyroY,int16_t* GyroZ);

#endif

