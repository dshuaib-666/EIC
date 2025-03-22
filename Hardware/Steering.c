#include "Steering.h"

//���PWM��ʼ��
void Steering_Init(void)
{
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
}

//���ռ�ձȷ�Χ(50,250)
void Steering_1(int Duty)
{
	__HAL_TIM_SetCompare(Steering_1_TIM,Steering_1_Channel,Duty);
}

void Steering_2(int Duty)
{
	__HAL_TIM_SetCompare(Steering_2_TIM,Steering_2_Channel,Duty);
}

void Steering_3(int Duty)
{
	__HAL_TIM_SetCompare(Steering_3_TIM,Steering_3_Channel,Duty);
}

void Steering_4(int Duty)
{
	__HAL_TIM_SetCompare(Steering_4_TIM,Steering_4_Channel,Duty);
}
 void Steering_frame_left(int Duty)
 {  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,Duty);
 
 }
  void Steering_frame_right(int Duty)
 {  __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,Duty); 
 
 }
int steering_value_l=60;
int steering_value_r=240;
int steering_value_time=0;
int steering_value_count=0;
void Steering_frame_below(void)
{   //steering_value_time++;
	Steering_frame_left(245); //左右平行
	    Steering_frame_right(55);

	
	
}
void Steering_frame_up(void)  //+-40
{        steering_value_time++;
		
		Steering_frame_left(80);
	   Steering_frame_right(220); //
	
}  
//范围约150-220//165是最好的抬头角度
void Steering_Camara(int Duty)
{
	//250为极低头，
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,Duty);
	//Steering_Camara(165);
}

void Steering_Camara_up(void) //21cha       169
{
	 Steering_Camara( 153);//头153
    // Steering_Camara( 165);//头
}
void Steering_Camara_below(void)             //190
{
	Steering_Camara( 168);//头
     //Steering_Camara( 186);//头
}
void Steering_Camara_below_more(void)  
{
	  Steering_Camara( 192);//头
	 // Steering_Camara( 203);//头

}	
//200为垂直，150为测试正常 越大越往内
void Steering_tongs_left (int Duty)
{
     __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,Duty);

}
 //120为垂直，150为测试正常越小越往内
void Steering_tongs_Right (int Duty)
{
     __HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,Duty);

} 
int  Steering_tongs_close_time=0;
void Steering_tongs_close(void)
{
//	Steering_tongs_close_time++; //2ms加一次
//		 if(Steering_tongs_close_time<=500) 
//		 {Steering_tongs_Right(190 - 0.17 * Steering_tongs_close_time);
//		Steering_tongs_left( 130 + 0.18 * Steering_tongs_close_time); }
//		 else{
		      	    Steering_tongs_Right(115);
					Steering_tongs_left(213);
		 
		 //} 
		
	

}

void Steering_tongs_loosen(void)//松开
{
		Steering_tongs_Right(170);
		Steering_tongs_left(150);        

}

void Steering_tongs_loosen_slight(void)//松开
{
		Steering_tongs_Right(130);
		Steering_tongs_left(190);

}
