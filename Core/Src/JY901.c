#include "stdint.h"
#include "jy901.h"
#include <string.h>
#include "MY_USART.h"
#include <math.h>
#include "usart.h"
#define RXBUFFER_LEN 33		//接收3类数据，一共33位

 User_USART JY901_data;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 	stcMag;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SQ stcQ;
float p0=0,p1=0,p2=0,p3=0;

char JIESUO[]={0XFF,0XAA,0X69,0X88,0XB5};
char ZERO[]={0XFF,0XAA,0X01,0X08,0X00};
char SIX[]={0XFF,0XAA,0X24,0X01,0X00};
char nine[]={0XFF,0XAA,0X24,0X00,0X00};
char z_zero[]={0XFF,0XAA,0X01,0X04,0X00};
char BAOCHUN[]={0XFF,0XAA,0X00,0X00,0X00};

void JY901_Process()
{
		if(JY901_data.Rx_len < RXBUFFER_LEN) return;   	//如果位数不对

		for(uint8_t i=0;i<4;i++)
		{
				if(JY901_data.RxBuffer[i*11]!= JY901_data.frame_head) return;	//如果帧头不对
				switch(JY901_data.RxBuffer[i*11+1])
				{
						case 0x51:	
							memcpy(&stcAcc,&JY901_data.RxBuffer[2 + i*11],8);
							for(uint8_t j = 0; j < 3; j++) JY901_data.acc.a[j] = (float)stcAcc.a[j]/32768*16;									//官方加速度解算
						break;
						case 0x52:	
							memcpy(&stcGyro,&JY901_data.RxBuffer[2 + i*11],8);
							for(uint8_t j = 0; j < 3; j++) JY901_data.w.w[j] = (float)stcGyro.w[j]/32768*2000;								//官方角速度解算
						break;
						case 0x53://角度	
							memcpy(&stcAngle,&JY901_data.RxBuffer[2 + i*11],8);
							for(uint8_t j = 0; j < 3; j++) JY901_data.angle.angle[j] = (float)stcAngle.Angle[j]/32768*180;		//官方角度解算
						break;
						case 0x59:	//四元数
							memcpy(&stcQ,&JY901_data.RxBuffer[2 + i*11],8);
							for(uint8_t j = 0; j < 4; j++) 
						JY901_data.q.q[j] = (float)stcQ.q[j]/32768;		
						break;	

				}
				
		}

}
//void JY901_q(float *p1,float *p2,float *p3,float *p4)
//{

//		*p1 = JY901_data.q.q[0]/  q30 ;
//		*p2= JY901_data.q.q[1] ;
//		*p3 = JY901_data.q.q[2];
//		*p4 = JY901_data.q.q[3];
//        
//      
//}

void JY901_GET(float *pitch,float *roll,float *yaw)
{

			*pitch=	JY901_data.angle.angle[0];
			*roll=JY901_data.angle.angle[1];
			*yaw=JY901_data.angle.angle[2];
}

void JY901_CONET(void)
{	
	
			HAL_UART_Transmit_DMA(&huart6,(uint8_t *)JIESUO,10);
			//使用6轴算法
			
			HAL_Delay(200);
			//xy置零       
			//HAL_UART_Transmit(&huart6,(uint8_t *)SIX,10,4);
			HAL_UART_Transmit_DMA(&huart6,(uint8_t *)ZERO,10);
			
	
			HAL_Delay(3000);
	
			HAL_UART_Transmit_DMA(&huart6,(uint8_t *)BAOCHUN,10);
	
}
//z轴归零
//需要停车3.2s
void JY901_CONET_z_zero(void)
{	//HAL_UART_Transmit(&huart1,0XFF,2,1);
			HAL_UART_Transmit(&huart2,(uint8_t *)JIESUO,10,10);
			
		
			HAL_Delay(200);
		
			HAL_UART_Transmit(&huart2,(uint8_t *)z_zero,10,10);
			//HAL_Delay(3000);
			HAL_UART_Transmit(&huart2,(uint8_t *)BAOCHUN,10,10);
		
}
//int n1=0;
//int jy1=0;
//extern int jy2;
//void jy901_4ms(void)
//{
//	n1++;//5ms加一次
//	if(n1==1&&jy1==0)
//	{
//		HAL_UART_Transmit(&huart2,(uint8_t *)JIESUO,10,4);
//	}
//	if(n1==2&&jy1==0){HAL_UART_Transmit(&huart2,(uint8_t *)SIX,10,4);}
//	if(n1 == 42 && jy1 == 0)//200ms
//	{
//		jy1 = 1;
//	}
//	if(jy1==1&&n1==43){HAL_UART_Transmit(&huart2,(uint8_t *)ZERO,10,4);}
//	if(jy1==1&&n1==44){HAL_UART_Transmit(&huart2,(uint8_t *)z_zero,10,4);}
//	if(n1==644&&jy1==1)//3000ms+44*5ms
//	{
//		jy1=2;
//	}
//	if(jy1==2)
//	{
//		HAL_UART_Transmit(&huart2,(uint8_t *)BAOCHUN,10,4);
//		jy1=3;
//		jy2=1;
//	}
//	
//}
