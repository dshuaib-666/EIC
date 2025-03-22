#include "OPENMV_COUNTER.h"
#include "openmv.h"
#include "PID_COUNTER.h"
#include "FRAME_COUNTER.h"
#include "location.h"
#include "usart.h"
#include "Steering.h"
#include "Motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "ATKTDF.h"
// OPEN_data为从openmv通过串口Uart3读取过来的数组
// 0为这帧中的人头的坐标x(x<320为左边x>320为右边）
// 1为y坐标
// 2为框的宽度
// 3为框的高度
// 4为框内的人数
// 5为充值好的偏移量
// 6为openmv的返回
// 1表示有人时；
// 2表示没人时；

//3.3
//舵机duoji及当前状态，
//当1时是（开局还没有抓到第一个球）大框抬起在找球（开局情况），并且钳子也是收起状态
//2是第一个球到了脚下，要伸出钳子夹球   注意第一次夹球不用大框
//3已经钳到第一个球，去找安全区
//
// 当4时是钳到的是错误的球，
// 当5时是钳到有正确的球去找安全区

// 6.7.8.9.6.7.8.9
// 当7是到了安全区，叫小车抬框并且送钳子且小车后退
// 除了第一次之后（通用）：
// 当6为摄像头看到安全区，再指示小车前往并且靠近安全区
//已经抬起来后为7，后退然后抬着框找球

//7之后duoji位回到8，也就是告诉小车回到寻球状态

//  当9摄像头发现球，则小车进入摄像头控制状态
//10是接近球（巨近），需要缓慢前进
// 11是脚底下有球 要停下来先钳住了，然后小车在钳住后1s放下大框
//12是大框的东西没有问题（不管是摸奖对了还是没框到）
// 当13是框到的是错误的，要把框抬起来走了
//14是钳到的是错的，则框和钳子都松开
//15进入小车粗略找安全区
//进入6-7-...



//3.9
//舵机duoji及当前状态，
//当1时是大框抬起在找球，并且钳子也是收起状态
//2是摄像头发现了要抓的物体，小车根据摄像头指示前进

//3是告诉小车要低头了，并且小车减速且低头完
 //4是小车低头之后，叫车钳球

//5是下框摸奖 并且小车发送串口‘y66’ 如果是第一次的话，树莓派直接跳到7/6
// 当6时是钳到的是错误的球，
// 当7时是钳到有正确的球去找安全区
//8是摄像头没有发现安全区，小车按照粗定位寻找安全区
//9为摄像头看到安全区，再指示小车前往并且靠近安全区
//当10是到了安全区，叫小车抬框并且送钳子且小车后退
//回到1
extern User_USART__openmv OPEN_data; // Uart4的DMA接收用户结构体
extern float X_real,Y_real;
void OPEN_X(int motor);
int frame=0,pliers=0;
int openmv_TASK_Temp=0;
uint8_t sa_openmv_buf[10];
int openmv_time_count_task=0;
extern HWT101_USART HWT101;
int openmv_time_count_yaw=0;
 extern LOCATION location;
void OPENMV_SPORT_TASK(	void)
{
	OPENMV_DATA_FITER();
    if(OPEN_data.duoji_FIFLTER ==1){
	     frame=0;
		 pliers=0;
		Steering_Camara_up();
		Steering_frame_up();
		Steering_tongs_loosen();
		// Steering_Camara( 169);//头
		// Location_Lookfor_wang();
		location_miidle_turn_common();
	//	Location_Lookfor_g();
		//Motor_370_stop();
		
		
	}
    if(OPEN_data.duoji_FIFLTER==2)
	{
	     frame=0;
		 pliers=0;
		 Steering_tongs_close();
		OPENMV_PID_COUNTER(9000);
	  // OPEN_X(8000);
		Steering_Camara_up();
		Steering_frame_up();
		//Steering_Camara( 169);//头
	}
	
	if(OPEN_data.duoji_FIFLTER==3)
	{      OPENMV_PID_COUNTER(5500);
		   Steering_frame_up();
		Steering_tongs_loosen_slight();
			frame=0;
		Steering_Camara_below();
			pliers=1;

	}
	
	
     if(OPEN_data.duoji_FIFLTER==4)
	{
						Steering_tongs_close();
			
						Steering_frame_up();
						Steering_Camara_below();
						Motor_370_stop();
				frame=1;
				pliers=1;
		ATK_tongs_COUNTER();
		openmv_time_count_task++;
		if(openmv_time_count_task%10==0){
		uint8_t sa_openmv_buf4 []= "a";
		

        HAL_UART_Transmit_DMA(&huart3,sa_openmv_buf4,6);	  
		}
			
				
		
	}
	  if(OPEN_data.duoji_FIFLTER==5)   // 当5时是下框
	{           Steering_tongs_close();
				Steering_frame_below();
		Steering_Camara_below();
		Motor_370_stop();
	      frame=0;
		 pliers=0;
		
		
	}
	if(OPEN_data.duoji_FIFLTER==6)     // 当6时是钳到的是错误的球，
	{
		 Steering_tongs_close();
		Steering_frame_up();
		 Steering_Camara_below_more();
		   Motor_370_stop();
	      frame=1;
		 pliers=1;
	}
	
	  if(OPEN_data.duoji_FIFLTER==7)     // 当6时是钳到的是错误的球，
	{
		 Steering_tongs_loosen();
		Steering_frame_up();
		 Steering_Camara_up();
	      frame=1;
		 pliers=1;
		 Motor_370_stop();
	}

	  if(OPEN_data.duoji_FIFLTER==8)   //钳到有正确的球去找安全区
	{
		Steering_Camara_up();
		Steering_tongs_close();
		Steering_frame_up();
	      frame=1;
		 pliers=1;
		 Location_go_home();
		openmv_time_count_task++;
		if(openmv_time_count_task%10==0){
		uint8_t sa_openmv_buf7 []= "b";
		

        HAL_UART_Transmit_DMA(&huart3,sa_openmv_buf7,6);}	
		
	} 
      if(OPEN_data.duoji_FIFLTER==9)
	{    // location_face_to_safe(110,120,X_real,Y_real);
		Steering_Camara_up();
		 Location_go_home();
		Steering_frame_up();
		
	      frame=0;
		 pliers=0;
		  //OPEN_X();
	}
	
	   if(OPEN_data.duoji_FIFLTER==10)
	{   
		Steering_Camara_up();		
		OPENMV_PID_COUNTER(7000);
		Steering_tongs_close();
		Steering_frame_up();
			//Motor_370_stop();
	      frame=1;
		 pliers=1;
		
	}
	
	
		if(OPEN_data.duoji_FIFLTER==11)
		{
			Steering_Camara_up();		
		OPENMV_PID_COUNTER(7000);
		Steering_tongs_close();
		Steering_frame_up();
		
		}
		if(OPEN_data.duoji_FIFLTER==12)
		{   
			if(openmv_time_count_yaw==0)
			{openmv_time_count_yaw++; 
			//HWT101.ball_angle=HWT101.angle;
			location.x_record=location.x;
			location.y_record=location.y;	
			}
			//retreat_PID_COUNTER(10000,  -HWT101.ball_angle);
			Steering_Camara_below();	
		  Steering_tongs_loosen();
		  Steering_frame_up();
		  Motor_370_advance(-6000);
			
		
		}
		
		if(OPEN_data.duoji_FIFLTER==13)
		{
			
			 location_miidle_turn_common();
			openmv_time_count_yaw=0;
			Steering_Camara_up();	
		  Steering_tongs_close();
		  Steering_frame_up();
		//Motor_Location_turn(5000);
		
		}
		
}
void OPEN_Steering(void)
{
    if (OPEN_data.duoji == 1 || OPEN_data.duoji == 7)
    {
        frame_on();
    }
    if (OPEN_data.duoji == 2 || OPEN_data.duoji == 8)
    {
        frame_slow_below();
    }
    if (OPEN_data.duoji == 3 || OPEN_data.duoji == 5)
    {
        frame_below();
    }
    if (OPEN_data.duoji == 4 || OPEN_data.duoji == 6 || OPEN_data.duoji == 9)
    {
        frame_slow_up();
    }
}

//void OPEN_conuter_sport(void)
//{
//    if (OPEN_data.duoji == 3)
//    {
//    }
//    if (OPEN_data.duoji == 5)
//    {
//    }
//}

float OPENMV_YAW = 0;

int sa_time1 = 0;
 int sa25 = 0, sa26 = 0;

uint16_t  sa_openmv_fliter[20];
int   sa_openmv_fliter_time=0;
float sa_openmv_x_sum;
float temp_sa_open=0.0f;

void OPEN_X(int motor)
{
	if(OPEN_data.RX_staus==2){
					Motor_370_stop();
	              return;
	}
	sa_openmv_fliter_time++;
	if(sa_openmv_fliter_time<10){
	       return;
	}
//	sa_openmv_fliter[sa_openmv_fliter_time]= OPEN_data.OPENMV_X;
//	if(sa_openmv_fliter_time>=10)
//	{
//        sa_openmv_fliter_time=0;
//        for(int k=0;k<10;k++)
//		{
//			for(int j=0;j<9-k;j++)
//			{
//				
//				if (sa_openmv_fliter[j] > sa_openmv_fliter[j + 1]) {
//                temp_sa_open = sa_openmv_fliter[j];
//                sa_openmv_fliter[j] = sa_openmv_fliter[j + 1];
//                sa_openmv_fliter[j + 1] = temp_sa_open;
//            }
//		     
//			
//			}
//		
//		
//		}
//		
//		for (int  i = 1; i < 9; i++) {
//        sa_openmv_x_sum += sa_openmv_fliter[i];
//    }
//		
//	    OPEN_data. OPENMV_X_FIFLTER= sa_openmv_x_sum/8.0f;
//		 sa_openmv_x_sum=0;
		
	   // OPENMV_YAW = -((9.0f) * yaw_sa) / 32.0f + 45.0f;
	
	
   // OPENMV_YAW = -((9.0f) *  OPEN_data. OPENMV_X_FIFLTER) / 32.0f + 45.0f; // 将openmv的X坐标转换为偏转角度，记得要调转X方向 X=0--yaw=45  X=160--yaw=0  X=320--yaw=-45
    OPENMV_YAW=30.0f-3.0f*OPEN_data.OPENMV_X/16.0f;
    straight_line(motor, motor, OPENMV_YAW, &sa25, &sa26);

	
	}
                         
   


/*
   //OPEN_Steering();//头部驱动运行状态
*/
