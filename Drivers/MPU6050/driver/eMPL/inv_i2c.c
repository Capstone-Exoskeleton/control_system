#include "i2c.h"  
#include "inv_i2c.h"  
#include "stdio.h"

HAL_StatusTypeDef i2c_write(unsigned char slave_addr, unsigned char reg_addr,
      unsigned char length, unsigned char const *data)
{
			return HAL_I2C_Master_Transmitx(NULL, slave_addr<<1, reg_addr, data, length, 0xffff);
}

HAL_StatusTypeDef i2c_read(unsigned char slave_addr, unsigned char reg_addr,
      unsigned char length, unsigned char *data)
			{
			if (HAL_I2C_Master_Transmitx(NULL, slave_addr<<1, reg_addr, data, 0, 0xffff) == HAL_OK) // point to register
				return HAL_I2C_Master_Receivex(NULL, (slave_addr<<1)|0x01/*for read*/, (uint8_t*)data, length, 0xffff);
			else
				return HAL_ERROR;
			}
