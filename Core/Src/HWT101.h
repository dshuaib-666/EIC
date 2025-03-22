#ifndef __HWT101_H
#define __HWT101_H


#include "stdint.h"
#include "MY_USART.h"
struct HWT101_Angle//�Ƕ�
{
	short Angle[3];
	short T;
};

struct HWT101_SGyro//���ٶ�
{
	short SGyro[3];
	short T;
};


void HWT101_data_reduction(void);
void HWT101_reset(void);//��Ҫ��

void HWT101_z_zero(void); // z�����
#endif



