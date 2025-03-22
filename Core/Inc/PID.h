
#ifndef __PID_H
#define __PID_H

typedef struct 
{
	float kp;        // ����ϵ��
	float ki;        // ����ϵ��
	float kd;        // ΢��ϵ��
}PID;

extern PID Location_PID;

int all_around_PID(float deviation,PID Any_PID);


//ƫ���ǵ�PID
int position_pid_for_yaw(float deviation);


//�ٶȻ�pid
int position_pid_for_SPEED(float deviation) ;

//�ǶȻ���pid
int position_pid_for_turn(float deviation) ;
 int position_pid_for_openmv(float deviation) ;
#endif
