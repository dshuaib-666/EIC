#include "Key.h"

int Key_1(void)
{
	int i = 0;
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7) == 0)
		i = 1;
	return i;
}

int Key_2(void)
{
	int i = 0;
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11) == 0)
		i = 1;
	return i;
}
