#include "Motor.h"
#include "HWT101.h"
#include "MY_USART.h"
extern HWT101_USART HWT101; //���ս��սṹ��
void Motor_Init()
{
	HAL_TIM_PWM_Start(Motor_370_Left_TIM,Motor_370_Left_Channel);//����PWM��ʼ��	
	HAL_TIM_PWM_Start(Motor_370_Right_TIM,Motor_370_Right_Channel);	//�ҵ��PWM��ʼ��
}



//�ҵ������ռ�ձ�(��ת)	
//��Χ��-16800,+16800��

void Motor_370_R(int Duty)
{
	if(Duty > 16800 && Duty < -16800)
	{Duty = 0;}
	if(Duty > 0)
	{
		HAL_GPIO_WritePin(L_Ain1_GPIO,L_Ain1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_Ain2_GPIO,L_Ain2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(Motor_370_Left_TIM,Motor_370_Left_Channel,Duty);		
			//370(��)��ת
		
	}
	else
	{
		HAL_GPIO_WritePin(L_Ain1_GPIO,L_Ain1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_Ain2_GPIO,L_Ain2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SetCompare(Motor_370_Left_TIM,Motor_370_Left_Channel,-Duty);
			//370(��)��ת
		
	}
}

//��������ռ�ձ�(��ת)	
//��Χ��-16800,+16800��

void Motor_370_L(int Duty)
{
	if(Duty > 16800 && Duty < -16800)
		Duty = 0;
	if(Duty > 0)
	{
			//370(��)��ת
			HAL_GPIO_WritePin(R_Bin1_GPIO,R_Bin1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(R_Bin2_GPIO,R_Bin2_Pin,GPIO_PIN_SET);
			__HAL_TIM_SetCompare(Motor_370_Right_TIM,Motor_370_Right_Channel,Duty);		
		   	
	}
	else
	{
			//370(��)��ת      
			HAL_GPIO_WritePin(R_Bin1_GPIO,R_Bin1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(R_Bin2_GPIO,R_Bin2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(Motor_370_Right_TIM,Motor_370_Right_Channel,-Duty);
		    
	}
}

//˫�ֱַ����
void Motor_370_respectively(int left_Duty,int right_Duty)
{
	//�ڵײ�����޷�
	if(left_Duty>15000){left_Duty=15000;}
	if(left_Duty<-15000){left_Duty=-15000;}
	if(right_Duty>15000){right_Duty=15000;}
	if(right_Duty<-15000){right_Duty=-15000;}
			Motor_370_L(left_Duty);
			Motor_370_R(right_Duty);


}
 //ǰ���ͺ�����ͣ���������ǰ�����������Ǻ���
void Motor_370_advance(int Duty)
{
			Motor_370_respectively(Duty,Duty);


}

 //ǰ���ͺ�����ͣ���������ǰ�����������Ǻ���
void Motor_370_stop(void)
{
			Motor_370_respectively(0,0);


}
//ԭ����ת���ҷ��ظտ�ʼ�ĽǶ�
float Motor_Location_turn(int motor)
{
	static int sa_turnn=0;
	static float first_yaw=0; 
	if(sa_turnn==0){   first_yaw=HWT101.angle;
	                     sa_turnn=1;
	}
    Motor_370_respectively(-motor,motor);
	return first_yaw;
}
