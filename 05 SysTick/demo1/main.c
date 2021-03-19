#include "stm32f4xx.h"

GPIO_InitTypeDef GPIO_InitStructure ;
EXTI_InitTypeDef EXTI_InitStructure ;
NVIC_InitTypeDef NVIC_InitStructure ;

void delay(void)
{

	uint32_t  i=0x2000000;
	while(i--);
}

#define PFout(n)	*((uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PEout(n)	*((uint32_t *)(0x42000000+(GPIOE_BASE+0x14-0x40000000)*32+(n)*4))


int main(void )
{
	//�򿪶˿�E��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�E����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE );
	
	//�򿪶˿�F��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�F����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE );
	
	//��ʼ����Ӧ�˿ڵ����� 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_10;//ָ����9������ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;//����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//�������ŵ���Ӧʱ��=1/100MHz .
	//�Ӹߵ�ƽ�л����͵�ƽ1/100MHz,�ٶ�Խ�죬���Ļ�Խ��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//��������ģʽ��������������͹����������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//��ʹ���ڲ�����������
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13|GPIO_Pin_14;//ָ����9������ 
	GPIO_Init(GPIOE ,&GPIO_InitStructure);
	
	PFout(9)=1;
	PFout(10)=1;	
	PEout(13)=1;
	PEout(14)=1;	

	//1���ڴ���1000���жϣ��ж�����T=1/f=1/1000Hz=1ms
	SysTick_Config(SystemCoreClock/1000);

	
	while(1)
	{
	

	}
	
	return 0 ;
}

void SysTick_Handler(void)
{
	static uint32_t i1=0;
	
	i1++;
	
	//500ms�ѵ�
	if(i1%100==0)
	{
		PFout(9)^=1;
	}

	if(i1%330==0)
	{
		PFout(10)^=1;
	}
	
	if(i1%1500==0)
	{
		PEout(13)^=1;
	}
	
	if(i1>=2200)
	{
		PEout(14)^=1;
		i1=0;
	}

}



