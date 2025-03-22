#ifndef __MY_USART_H
#define __MY_USART_H

#include "main.h"

#define RXBUFFER_LEN 33		//����3�����ݣ�һ��33λ jy901
#define HWT101_RXBUFFER_LEN 22     
#define ATKTDF_RXBUFFER_LEN 80    
typedef struct
{
	float angle[3];
}Angle;

typedef struct
{
	float a[3];
}Acc;

typedef struct
{
	float w[3];
}SGyro;


typedef struct//��Ԫ��
{ float q[4];
}SQ;

typedef struct//�ų����
{
	float h[3];
}SMag;

typedef struct//��ѹ�߶�
{
	float lPressure;
	float lAltitude;
}SPress;

typedef struct//��γ��
{
	float lLon;
	float lLat;
}SLonLat;

typedef struct User_USART
{
		uint8_t Rx_flag;											
		uint8_t Rx_len;												
		uint8_t frame_head;					//֡ͷ
		uint8_t RxBuffer[RXBUFFER_LEN];		//���ջ���
		Angle angle;						//�Ƕ�
		Acc acc;								//���ٶ�
		SGyro w;								//���ٶ�
		SMag h;									//�ų�
		SPress lPressure;   	  //��ѹ
		SPress lAltitude;     	//�߶�
		SLonLat lLon;						//����
		SLonLat lLat;						//ά��
		SQ q; 									//��Ԫ��
}User_USART;

 typedef struct HWT101_USART
{
		uint8_t Rx_flag;											
		uint8_t Rx_len;												
		uint8_t frame_head;					//֡ͷ
		uint8_t RxBuffer[HWT101_RXBUFFER_LEN];		//���ջ���
		float angle;						//�Ƕ�
	
		float w_Before_calibration;	//У׼ǰ�Ľ��ٶ�							//���ٶ�
	    float w_After_calibration;	//У׼��Ľ��ٶ�							//���ٶ�
		float ball_angle; 
		float turn_start_yaw;
}HWT101_USART;



 typedef struct ATKTDF_USART
{
		uint8_t Rx_flag;											
		uint8_t Rx_len;												
		uint8_t frame_head;					//??
		uint8_t RxBuffer[ATKTDF_RXBUFFER_LEN];		//???????
		float LOCATION;						//???
		float LOCATION_fifle;
		uint8_t middle_Buffer[30];
		int head_location;
		int ATK_STAUE;
	
	
}ATKTDF_USART;



 void HWT101_USART_Init(HWT101_USART *Data);
void User_USART_Init(User_USART *Data);
#endif
