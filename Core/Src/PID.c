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
	.kp=200,		//赋值比例值-80
	.ki=0,			//赋值积分值-10
	.kd=0,			//赋值微分值
};


PID speed_PID=
{
	.kp=1, //赋值比例值-80
	.ki=0,					//赋值积分值-10
	.kd=0,					//赋值微分值
};

PID turn_PID=
{
	.kp=500, //赋值比例值-80
	.ki=0,					//赋值积分值-10
	.kd=0,					//赋值微分值
};
 PID OPEN_PID=
{
	.kp=50, //爻值覉}值-80
	.ki=0,					//爻值芙貣值-10
	.kd=0,					//爻值微貣值
};
int PositionPID(float deviation,PID pid)
{
	float Position_KP=pid.kp,Position_KI=pid.ki,Position_KD=pid.kd;
	static float Bias,Pwm,Integral_bias,Last_Bias;
	Bias=deviation;                         		         //计算偏差
	Integral_bias+=Bias;	                               //求出偏差的积分
	Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias); //位置式PID控制器
	Last_Bias=Bias;                                      //保存上一次偏差 
	return Pwm;    
}

int all_around_PID(float deviation,PID Any_PID)
{
	return PositionPID(deviation,Any_PID);

}

// 调用PositionPID函数，传入对应PID实例
//偏航角的pid
int position_pid_for_yaw(float deviation) 
{
	return PositionPID(deviation, yaw_PID);
}
int position_pid_for_yaw_reverse(float deviation) 
{
	return PositionPID(deviation, yaw_PID);
}

//速度环的pid
int position_pid_for_SPEED(float deviation) 
{
  return PositionPID(deviation, speed_PID);
}

//角度环pid
 //速度环的pid
int position_pid_for_turn(float deviation) 
{
  return PositionPID(deviation, turn_PID);
}

 int position_pid_for_openmv(float deviation) 
 {
	 
	 
	 
	 
	 
	 
	 
	 
    return PositionPID(deviation, OPEN_PID);
 }
 