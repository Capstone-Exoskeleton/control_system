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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "cybergear.h"
#include "math.h"

#ifdef MPU6050_DRIVER
#include "i2c.h"
//#include "MPU6050.h"
#include "kalman.h"
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
#define MPU6050_DEVICE_ID 0x68
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

int fputc(int ch, FILE *f)//printf
{
	HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1,0xffff);
	return (ch);
}
state nextstate = IDLE;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==USER_KEY_Pin)
  {
		if (nextstate == Start_wait) nextstate = Slow_start;
		else nextstate = Slow_stop;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char str[30];
	float pitch = 0.0f;
	const float minTorque  = 0.5;
	const float maxTorque  = 4;
	float outTorque = 0;
	
	uint8_t stop_count = 0;
	uint8_t counter = 0;
	
	
	
	#ifdef MPU6050_DRIVER
	long data[9];
	uint8_t ID;
	#endif
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
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
  MX_UART5_Init();
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
	//Soft_I2C_Init();
	
	MPU6050_Init();
	while(MPU6050_GetID() != MPU6050_DEVICE_ID)
	{
		printf("Unable to verify MPU6050 ID: %d\n",ID);
		HAL_Delay(100);
		MPU6050_Init();
	}
#endif
	
	
	//Enable CAN2 power
	HAL_GPIO_WritePin(GPIOC, Power_OUT2_EN_Pin,1);
	HAL_Delay(10);

	
	
	//CAN start
	MX_CAN2_Filter_Init();
	HAL_TIM_Base_Start_IT(&htim2); // start timer for can2



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		
		HAL_Delay(5);
		
#ifdef MPU6050_DRIVER
		/*
		timestamp = HAL_GetTick();	//current time
		mpu_module_sampling();			//MPL sample rate
		if (mpu_read_euler(data, &timestamp))	
		{
		  printf("1: %ld, 2: %ld, 3: %ld",data[0], data[1], data[2]);
			pitch = 1.0f*data[0]/65536.f;					//Convert q16 format to degrees
			//sprintf(str,"%.2lf/%.2lf/%.2lf\r\n",roll, pitch, yaw);
			//CDC_Transmit_FS(str,30);
		}
		*/

		pitch = Get_kalman_angle() ;
#endif
		if (nextstate == Moter_init){
				stop_count = 0;
				init_cybergear(&mi_motor, 0x7F, Motion_mode);
			
				nextstate = Start_wait;
		}
		//slow start if this is the first
		if (nextstate == Slow_start){	
				start_cybergear(&mi_motor);
				
				//get the first angle
				if (pitch < 0 && pitch > -45 ){
						outTorque = minTorque;
				}
				//case 2: 0 < pitch < 150, output sin of angle
				else if (pitch >= 0 && pitch <= 150){
					outTorque = (maxTorque-minTorque) * sin(pitch / 180 * 3.14159) + minTorque;
					if (outTorque > maxTorque) outTorque = maxTorque;
				}
				//slow start
				for (float tmpTorque = 0; tmpTorque < outTorque; tmpTorque+=0.5){
						HAL_Delay(100);
						motor_controlmode(&mi_motor, tmpTorque, 0, 0, 0 , 0);
				}
				nextstate = Read_gyro;
		}
		else if (nextstate == Read_gyro){
				counter = 0;
				//angle to torque calculation
				//case 1: -45 < pitch < 0, output min torque
				if (pitch < 0 && pitch > -45 ){
					outTorque = minTorque;
					nextstate = Moter_output;
				}
				//case 2: 0 < pitch < 150, output sin of angle
				else if (pitch >= 0 && pitch <= 150){
					outTorque = (maxTorque-minTorque) * sin(pitch / 180 * 3.14159) + minTorque;
					if (outTorque > maxTorque) outTorque = maxTorque;
					nextstate = Moter_output;
				}
				//case 3: unsafe angle zone, jump to emergency stop
				else {
					nextstate= Slow_stop;
				}
		}
		else if (nextstate == Moter_output){
				motor_controlmode(&mi_motor, outTorque, 0, 0, 0 , 0);
		}
		/*
		else if (nextstate == Wait_response){
				counter++;
				if (counter == 0x40){
					stop_count++;
					if (stop_count == 3)
						nextstate = STOP;
					else
						nextstate = Read_gyro;
				}
		}
		*/
		else if (nextstate == Slow_stop){
				for (; outTorque>0; outTorque-=0.5){
						motor_controlmode(&mi_motor, outTorque, 0, 0, 0 , 0);
						HAL_Delay(100);
				}
				stop_cybergear(&mi_motor, 1);
				nextstate=Moter_init;
		}
		else if (nextstate == STOP){
				stop_cybergear(&mi_motor, 1);
				HAL_Delay(2000);
				nextstate=Moter_init;
		}

#ifdef HX711
		float weight;
		HAL_Delay(10);
		weight = hx711_weight(&loadcell, 10);
		//printf("w: %.2f\r\n",weight);
		sprintf(str,"w: %.2f\r\n",weight);
		CDC_Transmit_FS(str,15);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
