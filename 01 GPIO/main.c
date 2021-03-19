#include "stm32f4xx.h"

/* 
�����ܣ�KEY0-3����LED0-3������

*/
GPIO_InitTypeDef GPIO_InitStruct;

int main(void)
{
	// �򿪶˿�A��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�A����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	// �򿪶˿�F��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�F����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
	// �򿪶˿�E��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�E����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // ����Ϊ���ģʽ
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // �������ŵ���Ӧʱ��=1/100MHz���Ӹߵ�ƽ�л����͵�ƽ
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // �������ģʽ 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ���ڲ���������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; // ָ����9������
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10; // ָ����10������
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13; // ָ����13������
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14; // ָ����14������
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_SetBits(GPIOF, GPIO_Pin_9);
	GPIO_SetBits(GPIOF, GPIO_Pin_10);
	GPIO_SetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_14);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // ����Ϊ����ģʽ
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // KEY0
	GPIO_Init(GPIOA, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2; // KEY1
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3; // KEY2
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4; // KEY3
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	while(1)
	{
		// KEY0 -- LED0
		if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		{
			GPIO_ResetBits(GPIOF, GPIO_Pin_9);
		}
		else
		{
			GPIO_SetBits(GPIOF, GPIO_Pin_9);
		}
		
		// KEY1 -- LED1
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))
		{
			GPIO_ResetBits(GPIOF, GPIO_Pin_10);
		}
		else
		{
			GPIO_SetBits(GPIOF, GPIO_Pin_10);
		}
		
		// KEY2 -- LED2
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
		{
			GPIO_ResetBits(GPIOE, GPIO_Pin_13);
		}
		else
		{
			GPIO_SetBits(GPIOE, GPIO_Pin_13);
		}
		
		// KEY3 -- LED3
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
		{
			GPIO_ResetBits(GPIOE, GPIO_Pin_14);
		}
		else
		{
			GPIO_SetBits(GPIOE, GPIO_Pin_14);
		}
	}
	return 0;
}
