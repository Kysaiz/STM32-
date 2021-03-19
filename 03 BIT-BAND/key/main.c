#include "stm32f4xx.h"

/*
ʹ��λ����������סKEY1-3ʱLED���ƣ��ɿ�ʱ���
*/

GPIO_InitTypeDef GPIO_InitStruct;

#define		PFout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + x*4)

void delay(uint32_t delaytime)
{
	for(;delaytime>0;delaytime--);
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
		PFout(9) = PEin(2);
		PFout(10) = PEin(3);
		PEout(13) = PEin(4);
	}
	
}
