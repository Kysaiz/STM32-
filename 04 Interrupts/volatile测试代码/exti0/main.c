#include "stm32f4xx.h"
#include "sys.h"


//GPIO��ʼ���Ľṹ��
static GPIO_InitTypeDef  GPIO_InitStructure;

//EXTI��ʼ���Ľṹ��
static EXTI_InitTypeDef   EXTI_InitStructure;

//NVIC��ʼ���Ľṹ��
static NVIC_InitTypeDef   NVIC_InitStructure;

void delay(void)
{

	uint32_t i=0x5000000;
	
	while(i--);
}

//�����volatile 
static volatile uint32_t g_exti0_event=0;


int main(void)
{
	uint32_t a=0;
	uint32_t b=0;

	/* GPIOA GPIOE GPIOFӲ��ʱ��ʹ�ܣ�������GPIOA GPIOE GPIOF���� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	
	
	/* ʹ��ϵͳ����ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	

	
	
	/* ����PF9 PF10Ϊ���ģʽ����������ž�������ߵ͵�ƽ�Ĺ��� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;	//��9 10������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//�����������ǿ�������������ŵ������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//���ŵ��ٶ����Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//û��ʹ���ڲ���������
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOF,GPIO_Pin_9);

	
	/* ����PA0Ϊ����ģʽ����������ž���ʶ��ߵ͵�ƽ�Ĺ��� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				//��0������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//���ŵ��ٶ����Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//û��ʹ���ڲ���������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
		

	/* ��PA0�������ӵ��ⲿ�жϿ�����0*/
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* �����ⲿ�жϿ�����0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;				//ָ���ⲿ�жϿ�������ΪEXTI0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//�жϴ���
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //�����ش����������ͷŲŻᴥ���ж�
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//ʹ���ж�/����
	EXTI_Init(&EXTI_InitStructure);

	/* �����ⲿ�жϿ�����0�����жϣ����������ȼ� */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//�ⲿ�жϿ�����0���жϺ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;//��ռ���ȼ� 0x0F
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;		//��Ӧ���ӣ����ȼ�0x0F
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�/����
	NVIC_Init(&NVIC_InitStructure);
	
	PFout(9)=PFout(10)=1;
	
	a=g_exti0_event;	//0
	
	delay();
	PFout(9)=0;			//����ʱ���ҾͰ��°���1
	delay();
	PFout(9)=1;
	
	b=g_exti0_event;	//1
	
	if(a == b)			//a=0 b=1
	{
		PFout(10)=0;
		
		delay();
		
		PFout(10)=1;
		
		delay();

	
	}
	
	
	
	while(1)
	{
		

	}

}

//�жϷ�����
void EXTI0_IRQHandler(void)
{
	//����ⲿ�жϿ�����0���ж�״̬�Ƿ����ж�����
	if(EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		
		g_exti0_event++;

		/* ����ⲿ�жϿ�����0�ı�־������λ������CPU�Ѿ��������ж����󣬿��Դ����µ�һ���ж����� */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}


