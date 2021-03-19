#include "it.h"

static GPIO_InitTypeDef 	  GPIO_InitStructure;
static EXTI_InitTypeDef   	EXTI_InitStructure;
static NVIC_InitTypeDef   	NVIC_InitStructure;

void exti0_init(void)
{
	//ʹ��(��)�˿�A��Ӳ��ʱ�ӣ����ǶԶ˿�A����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	
	//ʹ��ϵͳ����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//����PA0����Ϊ����ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		//��0������
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN;		//����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//������Ӧ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//û��ʹ������������
	GPIO_Init(GPIOA,&GPIO_InitStructure);	


	//��PA0��EXTI0������һ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	//�ⲿ�жϵ�����
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;		//�ⲿ�ж�0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//�ж�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//�½��ش���������ʶ�𰴼��İ��£������ش��������ڼ�ⰴ�����ɿ���  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//ʹ��
	EXTI_Init(&EXTI_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//�ⲿ�ж�0������ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//��ռ���ȼ�0x02
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//��Ӧ���ȼ�0x03
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ�ܸ�ͨ��
	NVIC_Init(&NVIC_InitStructure);
	
}

void exti2_init(void)
{
	//ʹ��(��)�˿�E��Ӳ��ʱ�ӣ����ǶԶ˿�E����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	
	//ʹ��ϵͳ����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	
	
	//����PE2����Ϊ����ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		//��2������
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN;		//����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//������Ӧ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//û��ʹ������������
	GPIO_Init(GPIOE,&GPIO_InitStructure);	


	//��PE2��EXTI2������һ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);

	//�ⲿ�жϵ�����
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;		//�ⲿ�ж�2
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//�ж�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//�½��ش���������ʶ�𰴼��İ��£������ش��������ڼ�ⰴ�����ɿ���  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//ʹ��
	EXTI_Init(&EXTI_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//�ⲿ�ж�2������ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�0x03
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//��Ӧ���ȼ�0x03
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ�ܸ�ͨ��
	NVIC_Init(&NVIC_InitStructure);
	
}

void exti3_init(void)
{
	//ʹ��(��)�˿�E��Ӳ��ʱ�ӣ����ǶԶ˿�E����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	
	//ʹ��ϵͳ����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	
	
	//����PE3����Ϊ����ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		//��3������
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN;		//����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//������Ӧ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//û��ʹ������������
	GPIO_Init(GPIOE,&GPIO_InitStructure);	


	//��PE2��EXTI2������һ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);

	//�ⲿ�жϵ�����
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;		//�ⲿ�ж�3
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//�ж�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//�½��ش���������ʶ�𰴼��İ��£������ش��������ڼ�ⰴ�����ɿ���  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//ʹ��
	EXTI_Init(&EXTI_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//�ⲿ�ж�3������ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�0x03
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		//��Ӧ���ȼ�0x03
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ�ܸ�ͨ��
	NVIC_Init(&NVIC_InitStructure);
	
}

