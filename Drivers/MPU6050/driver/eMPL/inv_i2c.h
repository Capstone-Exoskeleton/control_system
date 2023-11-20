#ifndef INV_I2C_H
#define INV_I2C_H

#include "main.h"
//API for inv mpu
#define delay_ms    HAL_Delay
#define get_ms(timestamp) (*timestamp=HAL_GetTick())
#define log_i       printf
#define log_e       print
#define min(a,b) ((a<b)?a:b)

HAL_StatusTypeDef i2c_write(unsigned char slave_addr, unsigned char reg_addr,
      unsigned char length, unsigned char const *data);

HAL_StatusTypeDef i2c_read(unsigned char slave_addr, unsigned char reg_addr,
      unsigned char length, unsigned char *data);

#endif


