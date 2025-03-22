#ifndef __OPENMV_H
#define __OPENMV_H
 #include "stdint.h"
typedef struct User_USART__openmv
{
		uint8_t RxBuffer[60];
		int OPENMV_X;
	    float OPENMV_X_FIFLTER;
		uint8_t OPENMV_X_digit;
		uint8_t OPENMV_X_BUFFER[3];//缓存
		int OPENMV_Y;
		uint8_t OPENMV_Y_digit;
		uint8_t OPENMV_angle;//中心方向与小球的偏角，可导入角度环
		uint8_t location;//距离球大致距离，可以导入pid进行对应处理
		 int   area;
		int RX_staus;
		 int duoji; //舵机及当前状态，当1是为框抬起，当2是为框放下，当3时是框抬起在找球，当4时是框里有正确的球，当5时是下框了去找安全区
		int duoji_FIFLTER;
		uint8_t Rx_flag;											
		uint8_t Rx_len;												
		uint8_t frame_head;					//帧头
	//	uint8_t RxBuffer[RXBUFFER_LEN];		//接收缓冲
							
}User_USART__openmv;
 void openmv_data_Init(void);
 void openmv_data_get(void);
 #endif
