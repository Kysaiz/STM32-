#include "stm32f4xx.h"

/*
����Ƴ���
���ܣ�ͨ��KEY1-3���л�ʱ��
*/

GPIO_InitTypeDef GPIO_InitStruct;

void delay(uint32_t delaytime)
{
	for(;delaytime>0;delaytime--)
	{
		// KEY1 -> PLL
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))
		{
			RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
			RCC->CFGR |= RCC_CFGR_SW_PLL;
		}
		
		// KEY2 -> HSI
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
		{
			RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
			RCC->CFGR |= RCC_CFGR_SW_HSI;
		}
		
		// KEY3 -> HSE
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
		{
			RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
			RCC->CFGR |= RCC_CFGR_SW_HSE;
		}
	}
}

int main(void)
{
	// �򿪶˿�F��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�F����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
	// �򿪶˿�E��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�E����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // ����Ϊ���ģʽ
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // �������ŵ���Ӧʱ��=1/100MHz���Ӹߵ�ƽ�л����͵�ƽ
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // �������ģʽ 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ���ڲ���������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // ָ����9��10������
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14; // ָ����13��14������
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_SetBits(GPIOF, GPIO_Pin_9 |GPIO_Pin_10);
	GPIO_SetBits(GPIOE, GPIO_Pin_13 | GPIO_Pin_14);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // ����Ϊ����ģʽ

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4; // KEY1��2��3
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	while(1)
	{
		GPIO_SetBits(GPIOF, GPIO_Pin_9);
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);
		delay(0x50000);
		GPIO_SetBits(GPIOF, GPIO_Pin_10);
		GPIO_ResetBits(GPIOE, GPIO_Pin_13);
		delay(0x50000);
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
		GPIO_ResetBits(GPIOE, GPIO_Pin_14);
		delay(0x50000);
		GPIO_SetBits(GPIOE, GPIO_Pin_14);
		GPIO_ResetBits(GPIOE, GPIO_Pin_13);
		delay(0x50000);
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);
		delay(0x50000);
		GPIO_SetBits(GPIOF, GPIO_Pin_10);
		GPIO_ResetBits(GPIOF, GPIO_Pin_9);
		delay(0x50000);
	}
	
}
