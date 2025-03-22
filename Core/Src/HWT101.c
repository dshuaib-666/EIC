#include "HWT101.h"

#include "usart.h"
struct HWT101_SGyro  HWT101_sgyro;
struct HWT101_Angle  HWT101_angle;//��Ϊ�м��������
HWT101_USART HWT101; //���ս��սṹ��
char HWT101_JIESUO[]={0XFF,0XAA,0X69,0X88,0XB5};
char HWT101_RESET[]={0XFF,0XAA,0X00,0XFF,0X00};
char HWT101_BAOCHUN[]={0XFF,0XAA,0X00,0X00,0X00};
char HWT101_Z_ZERO[]={0XFF,0XAA,0X76,0X00,0X00};


void HWT101_reset(void) //��Ҫ��
{
     HAL_UART_Transmit(&huart2,(uint8_t *)HWT101_JIESUO,10,5);
      HAL_UART_Transmit(&huart2,(uint8_t *)HWT101_RESET,10,5);
      HAL_UART_Transmit(&huart2,(uint8_t *)HWT101_BAOCHUN,10,5);
      HAL_Delay(1000);
}
void HWT101_z_zero(void) // z�����
{
     HAL_UART_Transmit_DMA(&huart2,(uint8_t *)HWT101_JIESUO,10);
	HAL_Delay(200);
      HAL_UART_Transmit_DMA(&huart2,(uint8_t *)HWT101_Z_ZERO,10);
	
	HAL_Delay(3000);
      HAL_UART_Transmit_DMA(&huart2,(uint8_t *)HWT101_BAOCHUN,10);
      
}

uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type) 
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}
 int sa11=0;
int sa12=0;
void HWT101_data_reduction(void)
{
	
	if(HWT101.Rx_len < HWT101_RXBUFFER_LEN) return;   	//���λ������
		

       
      
        if (HWT101.RxBuffer[11] == 0x55 && HWT101.RxBuffer[12] == 0x53) {
            // ���ݲ��ִ����� 6 ��ʼ�����ֽ���ǰ�����ֽ��ں�
            uint8_t yaw_l = HWT101.RxBuffer[17];
            uint8_t yaw_h = HWT101.RxBuffer[18];
            int16_t yaw = (int16_t)((yaw_h << 8) | yaw_l);
 
            // �������������ֵת��Ϊ�Ƕ�
            float angle = ((float)yaw / 32768.0f) * 180.0f;
            HWT101.angle = angle;
           
        } 
		 if (HWT101.RxBuffer[0] == 0x55 && HWT101.RxBuffer[1] == 0x52) {
            // �������ٶ�����
            uint8_t wy_l = HWT101.RxBuffer[4];
            uint8_t wy_h = HWT101.RxBuffer[5];
            int16_t wy = (int16_t)((wy_h << 8) | wy_l);
            HWT101.w_Before_calibration = ((float)wy / 32768.0f) * 2000.0f;
 
            uint8_t wz_l = HWT101.RxBuffer[6];
            uint8_t wz_h = HWT101.RxBuffer[7];
            int16_t wz = (int16_t)((wz_h << 8) | wz_l);
            HWT101.w_After_calibration = ((float)wz / 32768.0f) * 2000.0f; 
    }
}



