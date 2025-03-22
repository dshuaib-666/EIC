/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "location.h"
#include "NRF.h"
#include "usart.h"
#include "NRF_COUNTER.h"
#include <stdio.h>
#include "string.h"
#include "Encoder.h"
#include "MOTOR.h"
#include "PID_COUNTER.h"
#include "OPENMV_COUNTER.h"
#include "i2c_hal.h"
#include "OPENMV.h"
extern HWT101_USART HWT101; //���ս��սṹ��
#include "ATKTDF.h"
#include "Steering.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int sat_1=0,sat_2=0,sat_3=0,sat_4=0;
extern uint8_t sa_buf[10];
extern float X_real,Y_real;
extern struct NRF_BUFFER NBF_BUFF; 
extern int lookforwangsa;
extern   int sa1,sa2;
int sa100=0,sa101=0;
extern int sa_key1;
int sa_key12=0;
extern User_USART__openmv OPEN_data; //将DMA的数据存入这个数组里
unsigned char i2c_buffer_now=0;
int freertos_sa_gekai=0;
extern ATKTDF_USART atktdf;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern User_USART__openmv OPEN_data; // Uart4的DMA接收用户结构体
osMutexDef(XANDY_Mutex);   // 定义互斥锁
SemaphoreHandle_t  XANDYMutex;       // 互斥锁句柄
/* USER CODE END Variables */
osThreadId XANDYGETTASK1Handle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId USATHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void XANDYGETTASKs(void const * argument);
void normal(void const * argument);
void remotecontrolTASK(void const * argument);
void openmvcounter(void const * argument);
void UARTTASK(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of XANDYGETTASK1 */
  osThreadDef(XANDYGETTASK1, XANDYGETTASKs, osPriorityRealtime, 0, 256);
  XANDYGETTASK1Handle = osThreadCreate(osThread(XANDYGETTASK1), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, normal, osPriorityNormal, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, remotecontrolTASK, osPriorityNormal, 0, 256);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, openmvcounter, osPriorityNormal, 0, 256);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of USAT */
  osThreadDef(USAT, UARTTASK, osPriorityNormal, 0, 128);
  USATHandle = osThreadCreate(osThread(USAT), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  vTaskSuspend(myTask03Handle);
 //+
 //vTaskSuspend(myTask04Handle);
 vTaskSuspend(myTask02Handle);



  XANDYMutex = osMutexCreate(osMutex(XANDY_Mutex));
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_XANDYGETTASKs */
/**
  * @brief  Function implementing the XANDYGETTASK1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_XANDYGETTASKs */
void XANDYGETTASKs(void const * argument)
{
  /* USER CODE BEGIN XANDYGETTASKs */
  /* Infinite loop */
  for(;;)
  {                      
	  
	
    xSemaphoreTake(XANDYMutex, osWaitForever);  // 获取互斥锁
   X_AND_Y_GET(&X_real,&Y_real);
//	

    xSemaphoreGive(XANDYMutex);  // 释放互斥锁
    sat_1++;
    NRF_decide();//在这里去读取NRF是否控制
	 
    if(NBF_BUFF.start==1)
    {
    vTaskSuspend(myTask04Handle); 
    vTaskResume(myTask03Handle);   
    vTaskSuspend(myTask02Handle);
    }
//	if((sa_key1!=0)&&(sa_key12==0))
//	{
//		sa_key12++;
//	  vTaskResume(myTask02Handle);
//	
//	}

	
    vTaskDelay(5);
  }
  /* USER CODE END XANDYGETTASKs */
}

/* USER CODE BEGIN Header_normal */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_normal */
void normal(void const * argument)
{
  /* USER CODE BEGIN normal */
  /* Infinite loop */
  for(;;)
  {
    sat_2++; 
  xSemaphoreTake(XANDYMutex, osWaitForever);  // 获取互斥锁
	  location_miidle_turn_common();
	  //Location_Lookfor_wang(); 
//	Location_turn_PID(2,-175,175);  
	// Location_Lookfor_baoan();
  xSemaphoreGive(XANDYMutex);  // 释放互斥锁+
   
    
   

    vTaskDelay(5);
  }
  /* USER CODE END normal */
}

/* USER CODE BEGIN Header_remotecontrolTASK */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_remotecontrolTASK */
void remotecontrolTASK(void const * argument)
{
  /* USER CODE BEGIN remotecontrolTASK */
  /* Infinite loop */
  for(;;)
  {
    
  	sat_3++; 
//	 
//      // Location_Lookfor_wang();

//     //location_face_to_safe();
//	 // Motor_370_respectively(8000,8000);
 NRF_COUNTER_START();   //只进行NRF运行
    vTaskDelay(5);
  }
  /* USER CODE END remotecontrolTASK */
}

/* USER CODE BEGIN Header_openmvcounter */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_openmvcounter */
void openmvcounter(void const * argument)
{
  /* USER CODE BEGIN openmvcounter */
  /* Infinite loop */
  for(;;)
  {
//	//  location_face_to_safe(110,120,X_real,Y_real);
		sat_4++;
	   xSemaphoreTake(XANDYMutex, osWaitForever);  // 获取互斥锁
	   OPENMV_SPORT_TASK();
	  xSemaphoreGive(XANDYMutex);  // 释放互斥锁+
	  //Location_Lookfor_wang(); 
	 
				
	  
   vTaskDelay(5);
  }
  /* USER CODE END openmvcounter */
}

/* USER CODE BEGIN Header_UARTTASK */
/**
* @brief Function implementing the USAT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UARTTASK */
void UARTTASK(void const * argument)
{
  /* USER CODE BEGIN UARTTASK */
  /* Infinite loop */
  for(;;)
  {  
  // sprintf((char *)sa_buf,"x%f",OPEN_data.OPENMV_X_FIFLTER);
 //  HAL_UART_Transmit_DMA(&huart4,sa_buf,6);
  
//  sprintf((char *)sa_buf,"y%d",lookforwangsa);
//  HAL_UART_Transmit_DMA(&huart4,sa_buf,2);
//	   HAL_UART_Transmit_DMA(&huart3,sa_buf,6);
//	
	  // Steering_tongs_loosen();
	  ATKTDF_DATA_FIFTER();
      ATKTDF_GET_add();
	
	  
  vTaskDelay(100);
   
	 
	 
		
	  
  }

  /* USER CODE END UARTTASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
