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
 uint8_t buf[10];//��������ת��
void ENCODER_READ(void)
{
	//5000�ٶ���5ms����6.5����  5500 7  6000 7.5 6500 8.1 7000 8.6  3000 3.5
	//-5000 -6.2 -5500 -7 6000 -7.5
	//�õ�ǧ���ٶ�ֵ=798.0173*5ms�ٶ�+15.6273 //�ȱ���ת��
    READ_SPEED();
//	 sprintf((char *)buf,"%f  ",location.);
//     HAL_UART_Transmit_DMA(&huart4,(uint8_t *)&buf,3); //ʹ��DMA���͸�����������Ϣ

}
 extern User_USART__openmv OPEN_data; //��DMA�����ݴ������������
 int sa_stuas=0;//��״̬λ
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
