
#ifndef __PID_H
#define __PID_H

typedef struct 
{
	float kp;        // 比例系数
	float ki;        // 积分系数
	float kd;        // 微分系数
}PID;

extern PID Location_PID;

int all_around_PID(float deviation,PID Any_PID);


//偏航角的PID
int position_pid_for_yaw(float deviation);


//速度环pid
int position_pid_for_SPEED(float deviation) ;

//角度环的pid
int position_pid_for_turn(float deviation) ;
 int position_pid_for_openmv(float deviation) ;
#endif
