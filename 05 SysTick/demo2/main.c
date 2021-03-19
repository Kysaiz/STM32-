#include "stm32f4xx.h"

GPIO_InitTypeDef GPIO_InitStructure ;
EXTI_InitTypeDef EXTI_InitStructure ;
NVIC_InitTypeDef NVIC_InitStructure ;



//#define PFout(n)	*((uint32_t *)(0x42000000+((uint32_t)(&GPIOF->ODR)-0x40000000)*32+(n)*4))
//#define PFout(n)	*((uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PFout(n)	*((uint32_t *)(0x42000000+(0x40021414-0x40000000)*32+(n)*4))




void delay_ms(uint32_t n)
{

	while(n--)
	{
		SysTick->CTRL = 0; // Disable SysTick���ر�ϵͳ��ʱ�������ȡ����
		SysTick->LOAD = 21000-1; 	// ��д����ֵ���������ǵ���ʱʱ��
		SysTick->VAL = 0; 			// ��ձ�־λ
		SysTick->CTRL = 1; 			// ѡ��21MHz��ʱ��Դ������ʼ��ϵͳ��ʱ������
		while ((SysTick->CTRL & 0x10000)==0);//�ȴ��������
		
	}
	SysTick->CTRL = 0; // Disable SysTick
	
}

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
	
	PFout(9)=1;
	PFout(10)=1;	

	
	

	
	while(1)
	{
	
		delay_ms(1000);
		
		PFout(9)^=1;
	}
	
	return 0 ;
}



