#include "PID.h"


//typedef struct PID _PID;
typedef struct PID _PID;


PID Location_PID=	
{
	.kp = 1,
	.ki = 0.2,
	.kd = 0.05,
};

PID yaw_PID=
{
	.kp=200,		//��ֵ����ֵ-80
	.ki=0,			//��ֵ����ֵ-10
	.kd=0,			//��ֵ΢��ֵ
};


PID speed_PID=
{
	.kp=1, //��ֵ����ֵ-80
	.ki=0,					//��ֵ����ֵ-10
	.kd=0,					//��ֵ΢��ֵ
};

PID turn_PID=
{
	.kp=500, //��ֵ����ֵ-80
	.ki=0,					//��ֵ����ֵ-10
	.kd=0,					//��ֵ΢��ֵ
};
 PID OPEN_PID=
{
	.kp=50, //سֵ҈}ֵ-80
	.ki=0,					//سֵֵܽؖ-10
	.kd=0,					//سֵ΢ֵؖ
};
int PositionPID(float deviation,PID pid)
{
	float Position_KP=pid.kp,Position_KI=pid.ki,Position_KD=pid.kd;
	static float Bias,Pwm,Integral_bias,Last_Bias;
	Bias=deviation;                         		         //����ƫ��
	Integral_bias+=Bias;	                               //���ƫ��Ļ���
	Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias); //λ��ʽPID������
	Last_Bias=Bias;                                      //������һ��ƫ�� 
	return Pwm;    
}

int all_around_PID(float deviation,PID Any_PID)
{
	return PositionPID(deviation,Any_PID);

}

// ����PositionPID�����������ӦPIDʵ��
//ƫ���ǵ�pid
int position_pid_for_yaw(float deviation) 
{
	return PositionPID(deviation, yaw_PID);
}
int position_pid_for_yaw_reverse(float deviation) 
{
	return PositionPID(deviation, yaw_PID);
}

//�ٶȻ���pid
int position_pid_for_SPEED(float deviation) 
{
  return PositionPID(deviation, speed_PID);
}

//�ǶȻ�pid
 //�ٶȻ���pid
int position_pid_for_turn(float deviation) 
{
  return PositionPID(deviation, turn_PID);
}

 int position_pid_for_openmv(float deviation) 
 {
	 
	 
	 
	 
	 
	 
	 
	 
    return PositionPID(deviation, OPEN_PID);
 }
 