#ifndef __STEERING_H__
#define __STEERING_H__

#include "stm32f4xx.h"                  // Device header
#include "tim.h"


#define Steering_1_TIM			&htim10
#define Steering_1_Channel		TIM_CHANNEL_1

#define Steering_2_TIM			&htim11
#define Steering_2_Channel		TIM_CHANNEL_1

#define Steering_3_TIM			&htim9
#define Steering_3_Channel		TIM_CHANNEL_1

#define Steering_4_TIM			&htim9
#define Steering_4_Channel		TIM_CHANNEL_2



void Steering_Init(void);
void Steering_1(int Duty);
void Steering_2(int Duty);
void Steering_3(int Duty);
void Steering_4(int Duty);

void Steering_frame(int Duty);
void Steering_Camara(int Duty);
void Steering_Camara_up(void);

void Steering_Camara_below(void);
void Steering_frame_up(void)  ;

void Steering_frame_below(void)   ;
void Steering_tongs_left (int Duty);
void Steering_tongs_Right (int Duty);
void Steering_tongs_loosen(void);//松开
void Steering_tongs_close(void);
void Steering_Camara_below_more(void);
void Steering_tongs_loosen_slight(void);//松开
#endif
