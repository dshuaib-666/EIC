#include "MY_USART.h"
 extern HWT101_USART HWT101;
void User_USART_Init(User_USART *Data)
{
		for(uint16_t i=0; i < RXBUFFER_LEN; i++)	{Data->RxBuffer[i] = 0;}
		Data->frame_head = 0x55;
		Data->Rx_flag = 0;
		Data->Rx_len = 0;
}

void HWT101_USART_Init(HWT101_USART *Data)
{
		for(uint16_t i=0; i < HWT101_RXBUFFER_LEN; i++)	{Data->RxBuffer[i] = 0;}
		Data->frame_head = 0x55;
		Data->Rx_flag = 0;
		Data->Rx_len = 0;
		HWT101.angle=0;
		
		
}
