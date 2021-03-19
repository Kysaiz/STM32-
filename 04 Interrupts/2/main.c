#include "stm32f4xx.h"

/*
�ж����ȼ�
KEY0����LED0����ռ���ȼ�0����Ӧ���ȼ�0
KEY1����LED1����ռ���ȼ�2����Ӧ���ȼ�1
KEY2����LED2����ռ���ȼ�2����Ӧ���ȼ�2
KEY3����LED3����ռ���ȼ�3����Ӧ���ȼ�2
*/
NVIC_InitTypeDef NVIC_InitStruct;
EXTI_InitTypeDef EXTI_InitStruct;
GPIO_InitTypeDef GPIO_InitStruct;

#define		PFout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + x*4)
#define		PAin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000)*32 + x*4)

void delay()
{
	int delaytime = 0x1000000;
	for(;delaytime>0;delaytime--);
}

void EXTI0_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		PFout(9) = 0;
		delay();
		PFout(9) = 1;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line0);		
	}

}

void EXTI2_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		PFout(10) = 0;
		delay();
		PFout(10) = 1;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line2);		
	}

}

void EXTI3_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line3) == SET)
	{
		PEout(13) = 0;
		delay();
		PEout(13) = 1;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line3);		
	}

}

void EXTI4_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line4) == SET)
	{
		PEout(14) = 0;
		delay();
		PEout(14) = 1;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line4);		
	}

}

int main(void)
{
	// �򿪶˿�F��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�F����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
	// �򿪶˿�E��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�E����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
	
	// �򿪶˿�A��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�A����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	// �ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // ����Ϊ���ģʽ
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // �������ŵ���Ӧʱ��=1/100MHz���Ӹߵ�ƽ�л����͵�ƽ
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // �������ģʽ 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ���ڲ���������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10; // ָ����9��10������
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14; // ָ����13��14������
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_SetBits(GPIOF, GPIO_Pin_9 |GPIO_Pin_10);
	GPIO_SetBits(GPIOE, GPIO_Pin_13 | GPIO_Pin_14);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // ����Ϊ����ģʽ

	GPIO_InitStruct.GPIO_Pin = GPIO_PinSource2 | GPIO_PinSource3 | GPIO_PinSource4; // KEY1��2��3
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // KEY0
	GPIO_Init(GPIOA, &GPIO_InitStruct);	

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	EXTI_InitStruct.EXTI_Line = EXTI_Line0; 
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; 
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;  
	EXTI_Init(&EXTI_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line2; 
	EXTI_Init(&EXTI_InitStruct);	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line3; 
	EXTI_Init(&EXTI_InitStruct);	

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource4);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line4; 
	EXTI_Init(&EXTI_InitStruct);

	while(1)
	{
		
	}
	
}
