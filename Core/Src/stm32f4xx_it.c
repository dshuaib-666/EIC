/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy901.h"
#include "MY_USART.h"
#include "HWT101.h"
#include "openmv.h"
#include "ATKTDF.h"
extern User_USART__openmv OPEN_data;
extern ATKTDF_USART atktdf;
#define BUFFER_SIZE  100 
#include "ATKTDF_USA.h"
#include "ATKTDF_USA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
 extern User_USART JY901_data;
 extern HWT101_USART HWT101; //���ս��սṹ��
 extern  volatile uint8_t rx_len ;  //??????????????
extern volatile uint8_t recv_end_flag ; //??????????????
uint8_t sa_buffer_us4[100];
/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */
 extern User_USART__openmv OPEN_data; //将DMA的数据存入这个数组里

extern     char openmv_Data_rebuffer[25];
int it_C_temp=0;
int it_C_temp1=0;
int it_C_temp2=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim5;

/* USER CODE BEGIN EV */
int  sa10=0;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef g_uart_handle;  
 volatile   uint32_t tmp_flag1 = 0;
volatile int rx_lenu4=0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
   HWT101_data_reduction();
  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
     uint32_t temp_flag = 0;
	uint32_t temp;
	temp_flag = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE);
	if((temp_flag!=RESET))																
	{
			sa10++;
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);
			temp = huart2.Instance->SR;   										
			temp = huart2.Instance->DR; 										
			HAL_UART_DMAStop(&huart2);   									
			temp = hdma_usart2_rx.Instance->NDTR; 	
		
					
			HWT101.Rx_len = HWT101_RXBUFFER_LEN-temp;  				
			
			HWT101_data_reduction();		
				
			HWT101.Rx_flag = 1;   											
	}
	HAL_UART_Receive_DMA(&huart2,HWT101.RxBuffer,HWT101_RXBUFFER_LEN);
	
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
     uint32_t tmp_flag = 0;
	uint32_t temp;
	it_C_temp1++;
	tmp_flag =__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); //获取IDLE标志位
	if((tmp_flag != RESET))//idle标志被置位
	{   it_C_temp++;
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//清除标志位
		
		HAL_UART_DMAStop(&huart3); //  停止DMA传输，防止
		temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// 获取DMA中未传输的数据个数   
		//temp  = hdma_usart1_rx.Instance->NDTR;// 读取NDTR寄存器，获取DMA中未传输的数据个数，
		rx_len =  BUFFER_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
		 openmv_data_get();
		
		if(rx_len==0)
		{
			it_C_temp2++;
						for(int o=0;o<=59;o++)
						{
					 
					   OPEN_data.RxBuffer[o]=0;
					
						}
						  for(int o=0;o<=39;o++)
						{
						openmv_Data_rebuffer[o]=0;
				   
						}
		
		}		
		if(rx_len>25)
		{
		    for(int o=0;o<=25;o++)
						{
					 
					   openmv_Data_rebuffer[o]=0;
					
						}
			for(int o=0;o<=59;o++)
						{
					 
					   OPEN_data.RxBuffer[o]=0;
					
						}			
		
		}
		recv_end_flag = 1;	// 接受完成标志位置1	
	 }
	 HAL_UART_Receive_DMA(&huart3,OPEN_data.RxBuffer,BUFFER_SIZE);
  
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	 
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
/* USER CODE BEGIN UART4_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
   tmp_flag =__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE); //获取IDLE标志位
	if((tmp_flag != RESET))//idle标志被置位
	{   it_C_temp++;
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);//清除标志位
		
		HAL_UART_DMAStop(&huart4); //  停止DMA传输，防止
		temp  =  __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);// 获取DMA中未传输的数据个数   
		//temp  = hdma_usart1_rx.Instance->NDTR;// 读取NDTR寄存器，获取DMA中未传输的数据个数，
		rx_lenu4 =  100 - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
	//	ATKTDF_DATA_analysis();
	//	recv_end_flag = 1;	
	   ATKTDF_DATA_analysis();
		
		for(int k=0;k<=79;k++)
							{
							  atktdf.RxBuffer[k]=0 ;
							
							}
		temp=0;
		tmp_flag=0;
	}// 接受完成标志位置1		
		
    
    HAL_UART_Receive_IT(&huart4, (uint8_t*)atktdf.RxBuffer, sizeof(atktdf.RxBuffer)); // ???? UART ????
		
				
	
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
      JY901_Process();
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */
     
  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
   uint32_t temp_flag = 0;
	uint32_t temp;
	temp_flag = __HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE);
	if((temp_flag!=RESET))																
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);
			temp = huart6.Instance->SR;   										
			temp = huart6.Instance->DR; 										
			HAL_UART_DMAStop(&huart6);   									
			temp = hdma_usart6_rx.Instance->NDTR; 	
		
		//temp = hdma_usart2_rx.Instance->CNDTR; 				
			JY901_data.Rx_len = RXBUFFER_LEN-temp;  				
			JY901_Process();							
			//JY901_GET(&pitch,&roll,&yaw);		
			JY901_data.Rx_flag = 1;   											
	}
	HAL_UART_Receive_DMA(&huart6,JY901_data.RxBuffer,RXBUFFER_LEN);
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
