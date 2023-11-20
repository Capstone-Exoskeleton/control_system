#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"


/* Includes ------------------------------------------------------------------*/


/* Function typedef -----------------------------------------------------------*/
/** 
 *  @brief MPU模块初始化函数
 *  @return 无
 *  @attention 
 */
int module_mpu_init(void);

/** 
 *  @brief 从MPU的FIFO中读取数据并处理。
           其中也包含了mpl库对温度变化的处理。
 *  @return 循环次数，用于优化
 *  @attention 应该至少为10ms执行一次
 */
int mpu_module_sampling(void);
	
/** 
 *  @brief 包装器，包装inv_get_sensor_type_euler()函数
 *  @return 1 if data was updated. 
 *  @attention 
 */
signed char mpu_read_euler(long *data, unsigned long *timestamp);

#endif

