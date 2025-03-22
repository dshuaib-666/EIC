/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "Encoder.h"
#include "Steering.h"
#include "Key.h"
#include "oled.h"
#include "SPORT.h"
#include "jy901.h"
#include "NRF.h"
#include "MY_USART.h"
#include "PID_COUNTER.h"
#include "NRF_COUNTER.h"
#include "location.h"
#include <stdio.h>
#include "PID.h"
#include "openmv.h"
#include "LED.h"
#include <math.h>
  #include "OPENMV_COUNTER.h"
#include "HWT101.h"
#include "ATKTDF.h"
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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
 extern User_USART JY901_data;
extern  HWT101_USART HWT101; //最终接收结构体
extern ATKTDF_USART atktdf;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  uint8_t u4_buf[1]={1};
  uint8_t sa_buf[10]="sa1\n";
  uint8_t cntr=0;
extern struct NRF_BUFFER NBF_BUFF;  //定义一个结构体来接收数据   
  int j=0;
  float X_real=0,Y_real=0;
  int sa3=0,sa4=0;
  int sa_key1=0;
float sa50=0.0f,sa51=0.0f;
  int sa_ka=0;
  int sa_string=120;
  int sa_time_str=0;
  int sa2888=0;   int sa2889=0;
   uint16_t id;
   float sa_yaw,sa_pitch,sa_roll,sa_x,sa_Rx_flag,sa1_real,sa2_real,sa_goal_x = 30,sa_goal_y = 60,sa_distance,sa_angle_diff,sa_Ki1 = 100,sa_Ki2 = 500,sa_speed_L,sa_speed_R;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  User_USART_Init(&JY901_data);
  HWT101_USART_Init(&HWT101);
 // JY901_CONET();
  HWT101_z_zero();
  NRF_BUFFER_Init();
//    OLED_Init();
//   OLED_Clear();
//  OLED_ShowNum(0,0,1,4,16);
// OLED_ShowNum(0,0,13,4,16);
  Encoder_Init();
 Steering_Init();
 Motor_Init();
 ATKTDF_Init();
 // OLED_Init();
  openmv_data_Init();  
   while(NRF24L01_Check())
 {
  	HAL_Delay(1000);
 }
     
	
      
   NRF24L01_RX_Mode();//设置为发送模式
    

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
				//	Steering_frame_below();
	//	Steering_Camara_below();
	  
	  
	  
	   Steering_frame_below();
	  

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
     if(GPIO_Pin==KEY1_Pin)
	 {
		       sa_key1++;
		 
		 
		 
		 
		 
		 
	 }
 
 
 
 
 }


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) // ???????????? USART1
  {
	
    
    HAL_UART_Receive_IT(&huart4, (uint8_t*)atktdf.RxBuffer, sizeof(atktdf.RxBuffer)); // ???? UART ????
  }
}

 
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim == &htim6)
//	{
//		Speed_left = (short)__HAL_TIM_GET_COUNTER(&htim4);
//		pulse_left += (short)__HAL_TIM_GET_COUNTER(&htim4);
//		__HAL_TIM_SET_COUNTER(&htim4, 0); 
//		
//		Speed_right = (short)__HAL_TIM_GET_COUNTER(&htim3);
//		pulse_right += (short)__HAL_TIM_GET_COUNTER(&htim3);
//		__HAL_TIM_SET_COUNTER(&htim3, 0);
//		
//		__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
//	}
//} 
/***********************0.05s(5ms)Tim6�ص�����**************************/
//int sa_time=0;
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim== (&htim6))
//  {
//	 
//		X_AND_Y_GET(&X_real,&Y_real);
//		NRF_COUNTER_START();

//	 // OPENMV_COUNTER();
//	  Location_Lookfor_wang();
//	    sa_time++; 
//	  //Location_turn_PID();   
////Location_go_home();	
//	  // Location_Lookfor();	
////	  if(sa_time%20==0)
////	  {
////	  sprintf((char *)sa_buf,"x%f",X_real);
////	  HAL_UART_Transmit_DMA(&huart4,sa_buf,4);}
////	  if(sa_time%30==0)
////	  {          
////		  sprintf((char *)sa_buf,"y%f",Y_real);
////	  HAL_UART_Transmit_DMA(&huart4,sa_buf,4);
////	  }
//    

//}}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
