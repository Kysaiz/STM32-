#include "stm32f4xx.h"

/*
����Ƴ���
ʹ��λ��������ͨ���жϿ���KEY1-3���л�ʱ��
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
	int delaytime = 0x100000;
	for(;delaytime>0;delaytime--);
}

// KEY0�л�PLLʱ��
void EXTI0_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line0);		
	}

}

// KEY1�л�HSIʱ��
void EXTI2_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_HSI;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line2);		
	}

}

// KEY2�л�HSEʱ��
void EXTI3_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line3) == SET)
	{
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_HSE;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line3);		
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

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn; // ָ��ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F; // ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;// ��Ӧ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; // ��ͨ��
	NVIC_Init(&NVIC_InitStruct);

	EXTI_InitStruct.EXTI_Line = EXTI_Line0; 
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; 
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;  
	EXTI_Init(&EXTI_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line2; 
	EXTI_Init(&EXTI_InitStruct);	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line3; 
	EXTI_Init(&EXTI_InitStruct);	

	while(1)
	{
		GPIO_SetBits(GPIOF, GPIO_Pin_9);
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);
		delay();
		GPIO_SetBits(GPIOF, GPIO_Pin_10);
		GPIO_ResetBits(GPIOE, GPIO_Pin_13);
		delay();
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
		GPIO_ResetBits(GPIOE, GPIO_Pin_14);
		delay();
		GPIO_SetBits(GPIOE, GPIO_Pin_14);
		GPIO_ResetBits(GPIOE, GPIO_Pin_13);
		delay();
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);
		delay();
		GPIO_SetBits(GPIOF, GPIO_Pin_10);
		GPIO_ResetBits(GPIOF, GPIO_Pin_9);
		delay();
	}
	
}
