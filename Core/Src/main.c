/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "cybergear.h"

#ifdef MPU6050_DRIVER
#include "i2c.h"
#include "MPU6050.h"
#endif

#ifdef HX711
#include "hx711.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
int fputc(int ch, FILE *f)//printf
{
	HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1,0xffff);
	return (ch);
}
*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char str[30];
	
	#ifdef MPU6050_DRIVER
	uint8_t ID;
	float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
	const uint16_t minTorque  = 10;
	const uint16_t maxTorque  = 10000;
	uint16_t outTorque = 0;
	#endif
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	state nextstate = IDLE;
	linked_state(&nextstate);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_CAN2_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	// write LED
	HAL_GPIO_WritePin(GPIOC, LED_Pin,0);
	HAL_Delay(10);
	
	HAL_GPIO_WritePin(GPIOC, Power_5V_EN_Pin,1);
	HAL_Delay(10);
	
#ifdef HX711
	//enable hx711
	hx711_init(&loadcell, GPIOC, HX711_CLK_Pin, GPIOC, HX711_DATA_Pin);
	hx711_coef_set(&loadcell, -438.6); // read afer calibration
	hx711_tare(&loadcell, 30);
#endif

	
#ifdef MPU6050_DRIVER
	//---------------initialize MPU6050-----------------------
	Soft_I2C_Init();
	
	while(!module_mpu_init()){HAL_Delay(20);};

	unsigned long timestamp;
#endif
	
	
	//Enable CAN2 power
	HAL_GPIO_WritePin(GPIOC, Power_OUT2_EN_Pin,1);
	HAL_Delay(10);

	
	
	//CAN start
	MX_CAN2_Filter_Init();
	HAL_TIM_Base_Start_IT(&htim2); // start timer for can2
	
	/* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
	{
		HAL_Delay(10);
		switch (nextstate){
			case IDLE:


				break;
			case Moter_init:
				
				init_cybergear(&mi_motor, 0x7F, Motion_mode);
			
				nextstate = Read_gyro;
			
				break;
			case Read_gyro:
				
				nextstate=Moter_output;
				break;
			case Moter_output:
				
				
			
				nextstate = Read_gyro;
				break;
				
			case STOP:
				
				
				break;
			default: 
				break;
			
		}


#ifdef HX711
		//motor_controlmode(&mi_motor, 1, 1, 1, 1 , 1);

		//Debug print format
		sprintf(str,"Debug %d\r\n",1);
		//CDC_Transmit_FS(str,15);
		HAL_Delay(10);
		float weight;
		HAL_Delay(10);
		weight = hx711_weight(&loadcell, 10);
		//printf("w: %.2f\r\n",weight);
		sprintf(str,"w: %.2f\r\n",weight);
		CDC_Transmit_FS(str,15);
#endif
		
#ifdef MPU6050_DRIVER
		timestamp = HAL_GetTick();	//current time
		mpu_module_sampling();			//MPL sample rate
		long data[9];
		if (mpu_read_euler(data, &timestamp))	
		{
			pitch = 1.0f*data[0]/65536.f;					//Convert q16 format to degrees
			roll  = 1.0f*data[1]/65536.f;
			yaw 	= 1.0f*data[2]/65536.f;
			outTorque =  maxTorque - pitch; //TODO: using only pitch as measurement
			sprintf(str,"%.2lf/%.2lf/%.2lf\r\n",roll, pitch, yaw);
			CDC_Transmit_FS(str,30);
		}
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
