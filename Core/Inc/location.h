#ifndef __LOCATION_H
#define __LOCATION_H
#include "MY_USART.h"


 typedef struct LOCATION
{
		float x;
		float x_relative;//相对值 x为总值，x减去x记录值为这个相对值 ，使用的时候都使用相对值
		float x_record;  //记录值
		float y_relative;//相对值
		float y_record;
	    float y;
		float speed_sum;
		float speed_l_now;
		float speed_R_now;
		float speed_change;
	
		float x_sum;
		float y_sum;
	    float yaw;
}LOCATION;


void X_AND_Y_GET(float *x,float *y);

void Location_Go_wang(float goal_x,float goal_y,float *Distance,float *Angle_Diff);


void location_GO(int expect_x,int expect_y,float current_x,float current_y)	;
int Location_turn_PID(int lookfor, int start_yaw, int ending_yaw) ;// 根据当前状态获取目标角度
void Location_GO_EYSE(float expect_x,float expect_y,float current_x,float current_y);
 int Location_turn(void);
int Location_turn_PID(int lookfor,int start_yaw,int ending_yaw);//�����ĸ�״̬���õ��������
  void Location_strike(void);
void Location_Lookfor(void);
void Location_go_home(void);
void Location_Lookfor_wang(void);
void location_face_to_safe(float expect_x, float expect_y, float current_x, float current_y);
void Location_Lookfor_g(void);
void Location_Lookfor_baoan(void);
 void location_miidle_turn_common(void);//中心旋转
#endif

