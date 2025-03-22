#ifndef __PID_COUNTER_H
#define __PID_COUNTER_H



void YAW_PID(int MOTORL,int MOTORR,float yaw_expect,int *desired_value_l,int *desired_value_r);
 
 void SPEED_PID(int MOTORL,int MOTORR,float yaw_expect, int *desired_value_l,int *desired_value_r);
void straight_line(int except_MOTORl,int except_MOTORR,float except_yaw, int *desired_value_l,int *desired_value_r);
 void direction_PID(int except_MOTORl,int except_MOTORR,float except_yaw,float current_yaw);

void OPENMV_PID_COUNTER(int morot2)	;
void OPENMV_DATA_FITER(void);

 void retreat_PID_COUNTER(int motor,float YAW_EXPECT)  ;//ºóÍË














#endif
