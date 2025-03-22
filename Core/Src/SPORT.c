#include "SPORT.h"
#include "Encoder.h"
#include "Motor.h"
#include "usart.h"
#include "stdio.h"
#include "NRF_COUNTER.h"  
#include "OPENMV_COUNTER.h"
 #include "PID_COUNTER.h"
 #include "location.h"
 #include "OPENMV.h"
extern float Speed_left,Speed_right;
extern float pulse_left,pulse_right;
 extern float X_real,Y_real;
 uint8_t buf[10];//用于数据转化
void ENCODER_READ(void)
{
	//5000速度在5ms下是6.5脉冲  5500 7  6000 7.5 6500 8.1 7000 8.6  3000 3.5
	//-5000 -6.2 -5500 -7 6000 -7.5
	//得到千倍速度值=798.0173*5ms速度+15.6273 //等比例转化
    READ_SPEED();
//	 sprintf((char *)buf,"%f  ",location.);
//     HAL_UART_Transmit_DMA(&huart4,(uint8_t *)&buf,3); //使用DMA发送给蓝牙串口信息

}
 extern User_USART__openmv OPEN_data; //将DMA的数据存入这个数组里
 int sa_stuas=0;//总状态位
 extern int lookforsa;

void OPENMV_COUNTER(void)
{    static int  lookforsa_sa=0;
	if(sa_stuas==0)	
	{
		if(OPEN_data.RX_staus==2)
		{Location_Lookfor();
		lookforsa_sa=0;}
	    if(OPEN_data.RX_staus==1)
		{
		if((lookforsa%2)==1&&lookforsa_sa==0){lookforsa_sa=1;lookforsa=lookforsa-1;}
		OPEN_X(8000);
		}
		
	}

	
}
