#include "led.h"
#include "bit.h"

void init_led(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  
  
  /* Configure PD0 and PD2 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  PFout(9) = 1;
  PFout(10) = 1;
  PEout(13) = 1;
  PEout(14) = 1;  
}

void led_on(uint8_t led_num){
  if(led_num == 1)
    PFout(9) = 0;
  if(led_num == 2)
    PFout(10) = 0;
  if(led_num == 3)
    PEout(13) = 0;
  if(led_num == 4)
    PEout(14) = 0;  
  
}

void led_off(uint8_t led_num){
  if(led_num == 1)
    PFout(9) = 1;
  if(led_num == 2)
    PFout(10) = 1;
  if(led_num == 3)
    PEout(13) = 1;
  if(led_num == 4)
    PEout(14) = 1;  
  
}

