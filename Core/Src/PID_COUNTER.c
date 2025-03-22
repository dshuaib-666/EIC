#include "PID_COUNTER.h"
#include "jy901.h"
#include "PID.h"
#include "MOTOR.h"
#include "Encoder.h"
#include "HWT101.h"
#include "MY_USART.h"
#include "openmv.h"
#include "OPENMV_COUNTER.h"
#include "LED.h"
#include <math.h>
#include "location.h"
extern HWT101_USART HWT101; //���ս��սṹ��
extern User_USART__openmv OPEN_data; //将DMA的数据存入这个数组里
float current_pitch,current_roll,current_yaw;
extern  LOCATION location;
void YAW_PID(int MOTORL,int MOTORR,float yaw_expect,int *desired_value_l,int *desired_value_r)
{
  

	int motor_change;
	
	float errol_yaw;
	
	*desired_value_l =MOTORL;
	*desired_value_r =MOTORR;
    //JY901_GET(&current_pitch,&current_roll,&current_yaw);
	errol_yaw=yaw_expect-HWT101.angle;//�õ����ֵ������pid���� 
	motor_change= position_pid_for_yaw(errol_yaw);


    //  *desired_value_l=*desired_value_l-motor_change;
    //  *desired_value_r=*desired_value_r+motor_change;
	
	 if(((yaw_expect -HWT101.angle)>=180)||((yaw_expect - HWT101.angle)<=-180))
	{
	

		 *desired_value_l=*desired_value_l+motor_change;
		 *desired_value_r=*desired_value_r-motor_change;

	} 
	if(((yaw_expect - HWT101.angle)<180)&&((yaw_expect - HWT101.angle)>-180))
	{
		*desired_value_l=*desired_value_l-motor_change;
		*desired_value_r=*desired_value_r+motor_change;

	}
	
	
	
	 //Motor_370_respectively(motorl,motorr);
}
 //5000�ٶ���5ms����6.5����  5500 7  6000 7.5 6500 8.1 7000 8.6  3000 3.5
	//-5000 -6.2 -5500 -7 6000 -7
	//�õ�ǧ���ٶ�ֵ=798.0173*5ms�ٶ�+15.6273 //��.5����ת��

 //6000,15.5,5000,12  8000 21
//
float motorl_true,motorr_true;
	float motorl_deviation,motorr_deviation;
	float motorl_change,motorr_change;
	//extern float sa103, sa104;
void SPEED_PID(int MOTORL,int MOTORR,float yaw_expect, int *desired_value_l,int *desired_value_r)
{
	// desired_value_l ��Ŀ�������ٶ�
	YAW_PID(MOTORL,MOTORR,yaw_expect,desired_value_l,desired_value_r);
  // int motorl,motorr;
	

	
	
	//READ_SPEED(&motorl_true,&motorr_true);//�õ���ǰ5ms�������ٶ� 
   // motorl_deviation=(float )*desired_value_l-(motorl_true*798.0173f+15.6273f);
 //  motorr_deviation=(float )*desired_value_r-(motorr_true*798.0173f+15.6273f);
	
    motorl_deviation=(float )*desired_value_l-(location.speed_l_now*333.3f+1000.0f);
    motorr_deviation=(float )*desired_value_r-(location.speed_R_now*333.3f+1000.0f);

  motorl_change= position_pid_for_SPEED(motorl_deviation);
  motorr_change= position_pid_for_SPEED(motorr_deviation);
  *desired_value_l=*desired_value_l+motorl_change;
  *desired_value_r= *desired_value_r+motorr_change;
	Motor_370_respectively(*desired_value_l,*desired_value_r);
}

 void straight_line(int except_MOTORl,int except_MOTORR,float except_yaw, int *desired_value_l,int *desired_value_r)
 {
     
	 SPEED_PID(except_MOTORl,except_MOTORR,except_yaw,desired_value_l,desired_value_r);
}
 //�ǶȻ�pid
float sa_yaw_pid=0;//ƫ��
int sa_yaw_motor=0;	
int sa_yaw_turn_l=0,sa_yaw_turn_r=0;

void direction_PID(int except_MOTORl,int except_MOTORR,float except_yaw,float current_yaw)
{
    sa_yaw_pid= except_yaw- current_yaw;//�õ�ƫ��
	
	
	sa_yaw_motor= position_pid_for_turn(sa_yaw_pid);
	
	 if(((except_yaw -HWT101.angle)>=180)||((except_yaw - HWT101.angle)<=-180))
	{
	

		 sa_yaw_turn_l=sa_yaw_motor*(except_MOTORl/16000.0f);
		 sa_yaw_turn_r=-sa_yaw_motor*(except_MOTORR/16000.0f);

	} 
	if(((except_yaw - HWT101.angle)<180)&&((except_yaw - HWT101.angle)>-180))
	{
		 sa_yaw_turn_l=-sa_yaw_motor;
		 sa_yaw_turn_r=sa_yaw_motor;

	}
	
	 if(sa_yaw_turn_l>10000){sa_yaw_turn_l=10000;}
	if(sa_yaw_turn_l<-10000){sa_yaw_turn_l=-10000;}
	if(sa_yaw_turn_r>10000){sa_yaw_turn_r=10000;}
	if(sa_yaw_turn_r<-10000){sa_yaw_turn_r=-10000;}
	
	
	
//	sa_yaw_turn_l= -sa_yaw_motor*(except_MOTORl/16000.0f);
//	sa_yaw_turn_r=  sa_yaw_motor*(except_MOTORl/16000.0f);
	Motor_370_respectively(sa_yaw_turn_l,sa_yaw_turn_r);
}
float openmv_bias=0.0f;
float OPENMV_MOTOR_L=0.0f,OPENMV_MOTOR_R=0.0f;
float OPENMV_MOTOR_change=0.0f;
 
extern uint16_t  sa_openmv_fliter[20];
uint16_t  sa_openmv_duoji_fliter[20];


extern int   sa_openmv_fliter_time;
int   sa_openmv_fliter_duojitime=0;
extern float sa_openmv_x_sum;
float sa_openmv_DUOJI_sum=0;
extern float temp_sa_open;
int count_duoji[256] = {0};
int duoji_maxCount = 0;
int duoji_result = 0;
void OPENMV_DATA_FITER(void)
{

  
	
	sa_openmv_duoji_fliter[sa_openmv_fliter_duojitime]=OPEN_data.duoji;
	sa_openmv_fliter_duojitime++; 	
	if(sa_openmv_fliter_duojitime>=6)
	{
		sa_openmv_fliter_duojitime=0;
		for(int k=0;k<=4;k++)
		{
		 count_duoji[sa_openmv_duoji_fliter[k]]++;
		}
		for (int i = 1; i < 50; i++) {
        if (count_duoji[i] > duoji_maxCount) {
            duoji_maxCount = count_duoji[i];
            OPEN_data.duoji_FIFLTER = i;
        }
		}
		for(int i=0;i<=255;i++)
		{  count_duoji[i]=0;
		
		}
		//for(int i=0;i<=10;i++){sa_openmv_duoji_fliter[i]=0;}		
		
		//OPEN_data.duoji_FIFLTER= round(sa_openmv_DUOJI_sum/4.0f);
		  sa_openmv_DUOJI_sum=0; 
		duoji_maxCount=0;
	}
	else{return ;}


}

void OPENMV_PID_COUNTER(int morot2)	
{
		
	if(OPEN_data.OPENMV_X>320){
	Motor_370_stop();
		LED_ON();  
		return;
	}

		
	
	
	 if(OPEN_data.OPENMV_X<=0){
	     return;
	 }
	 
	 sa_openmv_fliter_time++;
	sa_openmv_fliter[sa_openmv_fliter_time]= OPEN_data.OPENMV_X;
    
	 
	if(sa_openmv_fliter_time>=4)
	{
        sa_openmv_fliter_time=0;
        for(int k=0;k<4;k++)
		{
			for(int j=0;j<3-k;j++)
			{
				
				if (sa_openmv_fliter[j] > sa_openmv_fliter[j + 1]) {
                temp_sa_open = sa_openmv_fliter[j];
                sa_openmv_fliter[j] = sa_openmv_fliter[j + 1];
                sa_openmv_fliter[j + 1] = temp_sa_open;
            }
		     
			
			}
		
		
		}
		
	
		
		for (int  i = 1; i < 3; i++) {
        sa_openmv_x_sum += sa_openmv_fliter[i];
    }
		
	    OPEN_data. OPENMV_X_FIFLTER= sa_openmv_x_sum/2.0f;
		 sa_openmv_x_sum=0;
		
	//OPEN_data.OPENMV_X_FIFLTER=OPEN_data.OPENMV_X;
	
	
	
    openmv_bias=OPEN_data.OPENMV_X_FIFLTER-160.0f; //得到偏置
  OPENMV_MOTOR_change=  position_pid_for_openmv(openmv_bias);
	
   OPENMV_MOTOR_R=morot2- OPENMV_MOTOR_change;
   OPENMV_MOTOR_L= morot2+ OPENMV_MOTOR_change;
     Motor_370_respectively(OPENMV_MOTOR_L,OPENMV_MOTOR_R);


}}	
 	int motor_ll=0,motor_rr=0;
float YAW_change=0;
float  motor_change1=0;
void retreat_PID_COUNTER(int motor,float YAW_EXPECT)  //后退
{
	
	

	YAW_change=YAW_EXPECT-HWT101.angle;
	motor_change1= position_pid_for_yaw(YAW_change);
	
	 if(((YAW_change -HWT101.angle)>=180)||((YAW_change - HWT101.angle)<=-180))
	{
	

		 motor_ll=motor-motor_change1;
		 motor_rr=motor+motor_change1;

	} 
	if(((YAW_change - HWT101.angle)<180)&&((YAW_change - HWT101.angle)>-180))
	{
		motor_ll=motor+motor_change1;
		motor_rr=motor-motor_change1;

	}
	 if(motor_ll>12000){motor_ll=12000;}
	if(motor_ll<-12000){motor_ll=-12000;}
	if(motor_rr>12000){motor_rr=12000;}
	if(motor_rr<-12000){motor_rr=-12000;}
	  Motor_370_respectively(-motor_ll,-motor_rr);
	
	
	
}
