#include "stm32f4xx.h"

/*
����Ƴ���
ʹ��λ��������ͨ��KEY1-3���л�ʱ��
*/

#define		PFout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + x*4)
#define		PAin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000)*32 + x*4)

NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

GPIO_InitTypeDef GPIO_InitStruct;

void tim3_init(void)
{
	//ʹ�ܶ�ʱ��3Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = (10000/2)-1;				//��ʱʱ������ã�Ҳ�������ü���ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;				//���÷�Ƶֵ��ȷ����ʱ����ʱ��Ƶ��
	//TIM_TimeBaseStructure.TIM_ClockDivision = 0;				//ʱ�ӷ�Ƶ���ڶ��η�Ƶ������F407��û�и÷�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//���ϼ�����0->TIM_Period�ͻᴥ���ж�����
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	//ʹ�ܶ�ʱ���ж�
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	//���ö�ʱ�����ȼ�			
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;				//��ʱ��3���жϺ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//��Ӧ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ��
	NVIC_Init(&NVIC_InitStructure);
	
	//ʹ�ܶ�ʱ������
	TIM_Cmd(TIM3,ENABLE);
}

void TIM3_IRQHandler(void)
{
	//��鶨ʱ��3�Ƿ���ʱ������ж�
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		PFout(9) ^= 1;
		//��ձ�־λ
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	
	
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

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4; // KEY1��2��3
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // KEY0
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	tim3_init();
	
	PFout(9)=PFout(10)=PEout(13)=PEout(14)=1;
	while(1)
	{
	
	}
	
}
