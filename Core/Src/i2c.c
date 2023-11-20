/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "gpio.h"
#include "stdio.h"


/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c3;

/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PA8     ------> I2C3_SCL
    PB4     ------> I2C3_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PA8     ------> I2C3_SCL
    PB4     ------> I2C3_SDA
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/** Private macros -----------------------------------------------------------*/
/**
 * @name CPU Frequency
 * @{
 */
#define CPU_FREQUENCY_MHZ 8
/** @}*/
/**
 * @name Baudrate
 * @{
 */
#define I2C_BAUDRATE_DELAY_US      2/**< Delay in us*/
 
#define Delay_us(xx)  delay_xus(xx)

#define SDA_Dout_LOW()          HAL_GPIO_WritePin(I2C1_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET) 
#define SDA_Dout_HIGH()         HAL_GPIO_WritePin(I2C1_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET)
#define SDA_Data_IN()           HAL_GPIO_ReadPin(I2C1_GPIO_Port, I2C1_SDA_Pin)
#define SCL_Dout_LOW()          HAL_GPIO_WritePin(I2C1_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET) 
#define SCL_Dout_HIGH()         HAL_GPIO_WritePin(I2C1_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET)
#define SCL_Data_IN()           HAL_GPIO_ReadPin(I2C1_GPIO_Port, I2C1_SCL_Pin)
#define SDA_Write(XX)           HAL_GPIO_WritePin(I2C1_GPIO_Port, I2C1_SDA_Pin, (XX?GPIO_PIN_SET:GPIO_PIN_RESET))
/** Private constants --------------------------------------------------------*/
/** Public variables ---------------------------------------------------------*/

/** Private variables --------------------------------------------------------*/
static uint8_t Soft_Free_State = 1;
/** Private function prototypes ----------------------------------------------*/
static void delay_xus(__IO uint32_t nTime);
static void SDA_Output(void);
static void SDA_Input(void);
static void SCL_Output(void);
static void SCL_Input(void);
static void I2C_Init(void);
static void I2C_Start(void);
static void I2C_Stop(void);
static uint8_t I2C_Wait_Ack(void);
static void I2C_Ack(void);
static void I2C_NAck(void);
static void I2C_Send_Byte(uint8_t txd);
static uint8_t I2C_Read_Byte(uint8_t ack);
static uint8_t I2C_Bus_Free_Check(void);
/** Private user code --------------------------------------------------------*/

/** Private application code -------------------------------------------------*/
/*******************************************************************************
*
*       Static code
*
********************************************************************************
*/
/**
  ******************************************************************
  * @brief   Delay in us
  * @param   [in]nTime n us
  * @retval  None
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static void delay_xus(__IO uint32_t nTime)
{
    int old_val,new_val,val;
 
    if(nTime > 900)
    {
        for(old_val = 0; old_val < nTime/900; old_val++)
        {
            delay_xus(900);
        }
        nTime = nTime%900;
    }
 
    old_val = SysTick->VAL;
    new_val = old_val - CPU_FREQUENCY_MHZ*nTime;
    if(new_val >= 0)
    {
        do
        {
            val = SysTick->VAL;
        }
        while((val < old_val)&&(val >= new_val));
    }
    else
    {
        new_val +=CPU_FREQUENCY_MHZ*1000;
        do
        {
            val = SysTick->VAL;
        }
        while((val <= old_val)||(val > new_val));
    }
}
 
/**
 ******************************************************************
 * @brief   I2C SDA Set Output Mode
 * @param   [in]None
 * @retval  None
 * @author  Yuying
 * @version V1.0
 * @date    2023/11/18
 ******************************************************************
 */
static void SDA_Output(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C1_GPIO_Port, &GPIO_InitStruct);
}
 
/**
 ******************************************************************
 * @brief   I2C SDA Set Input Mode
 * @param   [in]None
 * @retval  None
 * @author  Yuying
 * @version V1.0
 * @date    2023/11/18
 ******************************************************************
 */
static void SDA_Input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C1_GPIO_Port, &GPIO_InitStruct);
}
 
/**
 ******************************************************************
 * @brief   I2C SCL Set Output Mode
 * @param   [in]None
 * @retval  None
 * @author  Yuying
 * @version V1.0
 * @date    2023/11/18
 ******************************************************************
 */
static void SCL_Output(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = I2C1_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C1_GPIO_Port, &GPIO_InitStruct);
}
 
/**
 ******************************************************************
 * @brief   I2C SCL Set Input Mode
 * @param   [in]None
 * @retval  None
 * @author  Yuying
 * @version V1.0
 * @date    2023/11/18
 ******************************************************************
 */
static void SCL_Input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = I2C1_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C1_GPIO_Port, &GPIO_InitStruct);
}

/**
  ******************************************************************
  * @brief   I2C Init
  * @param   [in]None
  * @retval  None
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static void I2C_Init(void)
{  
  SCL_Dout_HIGH();
  SDA_Dout_HIGH();
  SCL_Output();
  SDA_Output();
}
 
/**
  ******************************************************************
  * @brief   I2C Start Signal
  * @param   [in]None
  * @retval  None
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static void I2C_Start(void)
{
  SDA_Dout_HIGH();
  SDA_Output();

  SCL_Dout_HIGH();
  Delay_us(I2C_BAUDRATE_DELAY_US);
  SDA_Dout_LOW();
  Delay_us(I2C_BAUDRATE_DELAY_US);
  SCL_Dout_LOW();
}
 
/**
  ******************************************************************
  * @brief   I2C Stop Signal
  * @param   [in]None
  * @retval  None
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static void I2C_Stop(void)
{
  SDA_Dout_LOW();
  SDA_Output();

  SCL_Dout_LOW();
  Delay_us(I2C_BAUDRATE_DELAY_US);
  SCL_Dout_HIGH();

  SDA_Dout_HIGH();
  Delay_us(I2C_BAUDRATE_DELAY_US);                 
}
 
/**
  ******************************************************************
  * @brief   I2C Wait For ACK
  * @param   [in]None
  * @retval  true Success（SDA low）
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static uint8_t I2C_Wait_Ack(void)
{
  uint8_t ucErrTime = 0;

  //SDA_Input();

  Delay_us(I2C_BAUDRATE_DELAY_US);

  //SDA_Dout_HIGH();Delay_us(1);     
  SCL_Dout_HIGH();Delay_us(1);

  while(SDA_Data_IN())
  {
    /*Timeout*/
    ucErrTime++;
    if(ucErrTime > (I2C_BAUDRATE_DELAY_US))
    {
      I2C_Stop();
      return 0;
    }
    Delay_us(1);
  }
  Delay_us(I2C_BAUDRATE_DELAY_US-ucErrTime);
  SCL_Dout_LOW();      
  return 1; 
}
 
/**
  ******************************************************************
  * @brief   I2C ACK
  * @param   [in]None
  * @retval  None
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static void I2C_Ack(void)
{
  SCL_Dout_LOW();

  SDA_Dout_LOW();
  SDA_Output();

  Delay_us(I2C_BAUDRATE_DELAY_US);
  SCL_Dout_HIGH();
  Delay_us(I2C_BAUDRATE_DELAY_US);
  SCL_Dout_LOW();
}
 
/**
  ******************************************************************
  * @brief   I2C Wait NACK
  * @param   [in]None
  * @retval  None
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static void I2C_NAck(void)
{
  SCL_Dout_LOW();

  SDA_Dout_HIGH();
  SDA_Output();

  Delay_us(I2C_BAUDRATE_DELAY_US);
  SCL_Dout_HIGH();
  Delay_us(I2C_BAUDRATE_DELAY_US);
  SCL_Dout_LOW();
}                        

/**
  ******************************************************************
  * @brief   I2C Send 1 Byte
  * @param   [in]txd data
  * @retval  None
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static void I2C_Send_Byte(uint8_t txd) 
{                        
  uint8_t t;      

  SCL_Dout_LOW();

  SDA_Dout_HIGH();
  SDA_Output();

  for(t = 0; t < 8; t++)
  {  
    SDA_Write((txd&0x80)>>7);       
    txd <<= 1;
    Delay_us(I2C_BAUDRATE_DELAY_US);
    SCL_Dout_HIGH();
    Delay_us(I2C_BAUDRATE_DELAY_US);   
    SCL_Dout_LOW();
  }   
}       

/**
  ******************************************************************
  * @brief   I2C Read 1 Byte
  * @param   [in]ack when ack=1，senf ACK，when ack=0，send nACK 
  * @retval  Data
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static uint8_t I2C_Read_Byte(uint8_t ack)
{
  uint8_t i, receive = 0;
  //SDA set input mode
  SDA_Input();
  for(i = 0; i < 8; i++)
  {
    SCL_Dout_LOW();
    Delay_us(I2C_BAUDRATE_DELAY_US);
    SCL_Dout_HIGH();
    receive <<= 1;
    if(SDA_Data_IN())receive++;   
    Delay_us(I2C_BAUDRATE_DELAY_US);
  }           
  if(!ack)I2C_NAck();/**< nACK*/
  else  I2C_Ack();    /**< ACK*/   

  return receive;
}

/**
  ******************************************************************
  * @brief   I2C State 
  * @param   [in]None.
  * @retval  True Free state
  * @author  Yuying
  * @version V1.0
  * @date    2023/11/18
  ******************************************************************
  */
static uint8_t I2C_Bus_Free_Check(void)
{
  uint8_t State = 0;
  //SDA input mode
  SDA_Input();

  /*SCL input mode*/
  SCL_Input();

  if(SDA_Data_IN() && SCL_Data_IN())
  {
    State = 1;
  }
  SCL_Output();
  return State;
}
/** Public application code --------------------------------------------------*/
/*******************************************************************************
*
*       Public code
*
********************************************************************************
*/
/**
  ******************************************************************
  * @brief   I2C Sned
  * @param   [in]x Null
  * @param   [in]Saddr Slave address
  * @param   [in]Raddr Register address
  * @param   [in]Buf Data
  * @param   [in]Size in byte
  * @param   [in]Timeout
  * @return  HAL_OK Success
  * @author  Yuying
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
HAL_StatusTypeDef HAL_I2C_Master_Transmitx(void *x, uint8_t Saddr, uint8_t Raddr, const uint8_t *Data, uint16_t Size, uint32_t Block_Time)
{
  (void)(Block_Time);
  /*I2C State*/
  if(I2C_Bus_Free_Check() == 0)
  {
    return 0;
  }

  Soft_Free_State = 0;
  I2C_Start();

  /*Slave Address*/
  I2C_Send_Byte(Saddr);
  if(I2C_Wait_Ack() == 0)
  {
    Soft_Free_State = 1;
    return HAL_ERROR;
  }

  I2C_Send_Byte(Raddr);
  if(I2C_Wait_Ack() == 0)
  {
    Soft_Free_State = 1;
    return HAL_ERROR;
  }

  for(uint16_t i = 0; i < Size; i++)
  {
    I2C_Send_Byte(Data[i]);
    if(I2C_Wait_Ack() == 0)
    {
      Soft_Free_State = 1;
      return HAL_ERROR;
    }
  }
  I2C_Stop();
  Soft_Free_State = 1;
  return HAL_OK;
}

/**
  ******************************************************************
  * @brief   I2C Receive
  * @param   [in]x Null
  * @param   [in]Saddr Slave address
  * @param   [in]Buf Data
  * @param   [in]Size in byte
  * @param   [in]Timeout
  * @return  HAL_OK Success
  * @author  Yuying
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
HAL_StatusTypeDef HAL_I2C_Master_Receivex(void *x, uint8_t Saddr, uint8_t *Buf, uint16_t Size, uint32_t Block_Time)
{
  (void)(Block_Time);
  /*Check I2C State*/
  if(I2C_Bus_Free_Check() == 0)
  {
    return 0;
  }  

  Soft_Free_State = 0;
  I2C_Start();

  /*Slave Address*/
  I2C_Send_Byte(Saddr);
  if(I2C_Wait_Ack() == 0)
  {
    Soft_Free_State = 1;
    return HAL_ERROR;
  }
	
  for(uint16_t i = 0; i < Size; i++)
  {
    Buf[i] = I2C_Read_Byte(((i+1) == Size)?0:1);
  }
	I2C_NAck(); // stop reciving
  I2C_Stop();
  Soft_Free_State = 1;
  return HAL_OK;
}

/**
  ******************************************************************
  * @brief   Check Free State
  * @param   [in]None.
  * @return  true Free.
  * @author  Yuying
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
uint8_t Soft_I2C_Is_Free(void)
{
  return Soft_Free_State;
}

/**
  ******************************************************************
  * @brief   Init I2C
  * @param   [in]None.
  * @return  None.
  * @author  Yuying
  * @version v1.0
  * @date    2023/11/18
  ******************************************************************
  */
void Soft_I2C_Init(void)
{
  I2C_Init();
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
