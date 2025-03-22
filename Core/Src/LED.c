#include "LED.h"
#include "gpio.h"

void LED_ON(void)
{
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

}
