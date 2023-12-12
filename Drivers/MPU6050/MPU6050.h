#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"


/* Includes ------------------------------------------------------------------*/


/* Function typedef -----------------------------------------------------------*/
/** 
 *  @brief MPU Initialization
 *  @return 1->success, 0->fail
 *  @attention 
 */
int module_mpu_init(void);

/** 
 *  @brief Read and process FIFO from MP.
 *  @return cycletime
 *  @attention Process at least every 10ms
 */
int mpu_module_sampling(void);
	
/** 
 *  @brief inv_get_sensor_type_euler() interface
 *  @return 1 if data was updated. 
 *  @attention 
 */
signed char mpu_read_euler(long *data, unsigned long *timestamp);

#endif

