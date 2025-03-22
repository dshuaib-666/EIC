#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx.h"                  // Device header
#include "tim.h"
#include "gpio.h"


//TIMͨ����ʼ������
#define Motor_370_Right_TIM			&htim1
#define Motor_370_Right_Channel			TIM_CHANNEL_4
                                      
#define Motor_370_Left_TIM			&htim1	
#define Motor_370_Left_Channel		     TIM_CHANNEL_2
	

//��������ת����
#define R_Bin1_GPIO		GPIOE
#define R_Bin1_Pin		 GPIO_PIN_15

#define R_Bin2_GPIO		GPIOE
#define R_Bin2_Pin		 GPIO_PIN_12
                          
#define L_Ain1_GPIO		GPIOE
#define L_Ain1_Pin		 GPIO_PIN_13

#define L_Ain2_GPIO		GPIOE
#define L_Ain2_Pin		  GPIO_PIN_10
                          

float Motor_Location_turn(int motor);
void Motor_Init(void);
void Motor_370_R(int Duty);
void Motor_370_L(int Duty);
void Motor_370_respectively(int left_Duty,int right_Duty);
void Motor_370_advance(int Duty);
void Motor_370_stop(void);

#endif
