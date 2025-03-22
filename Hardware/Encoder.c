#include "Encoder.h"
#include "location.h"
float Speed_left,Speed_right;
float pulse_left,pulse_right;
 uint32_t tick=0;
 extern  LOCATION location;
void Encoder_Init(void)
{
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);	
	
	__HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);//�������жϿ���
	

}
int Read_Speed(TIM_HandleTypeDef *htim)
{ 
	
	int temp;
	temp=(short)__HAL_TIM_GetCounter(htim);
	
	__HAL_TIM_SetCounter(htim,0);
	return temp;
	
}
//�õ���ǰ�ٶ�
void READ_SPEED(void)
{    
	

	
	

//	*Encoder_Left=(float)Read_Speed(&htim4);
//	*Encoder_Right=(float)Read_Speed(&htim3);
	location.speed_l_now=(float)Read_Speed(&htim4);
	location.speed_R_now=(float)Read_Speed(&htim3);
	
}
float sa5=0,sa6=0;
float sa7=0,sa8=0;
//���ڼ��һȦ��������
//1969 1992
void READ_PULSE_SUM(void)
{  READ_SPEED();
    sa7=location.speed_l_now+sa7;
	sa8=location.speed_R_now+sa8;
}

