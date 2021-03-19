#include "stm32f4xx.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_InitStructure ;
EXTI_InitTypeDef EXTI_InitStructure ;
NVIC_InitTypeDef NVIC_InitStructure ;
USART_InitTypeDef USART_InitStructure ;

#define PFout(n)	*((uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PEout(n)	*((uint32_t *)(0x42000000+(GPIOE_BASE+0x14-0x40000000)*32+(n)*4))

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);

	return ch;
}

void _sys_exit(int return_code) {
label: goto label; /* endless loop */
}

void delay_us(uint32_t n)
{
	SysTick->CTRL = 0; // Disable SysTick���ر�ϵͳ��ʱ�������ȡ����
	SysTick->LOAD = 21*n-1; 	// ��д����ֵ���������ǵ���ʱʱ��
	SysTick->VAL = 0; 			// ��ձ�־λ
	SysTick->CTRL = 1; 			// ѡ��21MHz��ʱ��Դ������ʼ��ϵͳ��ʱ������
	while ((SysTick->CTRL & 0x10000)==0);//�ȴ��������
	SysTick->CTRL = 0; // ����ʹ�þ͹ر�ϵͳ��ʱ��
}


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
	SysTick->CTRL = 0; // ����ʹ�þ͹ر�ϵͳ��ʱ��
}


void usart1_init(uint32_t baud)
{
	//�򿪶˿�AӲ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE );
	
	//�򿪴���1��Ӳ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );	
	
	//����PA9��PA10���ţ�ΪAFģʽ�����ù���ģʽ��
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_10;//ָ����9������ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//����Ϊ���ù���ģʽ
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//�������ŵ���Ӧʱ��=1/100MHz .
	//�Ӹߵ�ƽ�л����͵�ƽ1/100MHz,�ٶ�Խ�죬���Ļ�Խ��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//��������ģʽ��������������͹����������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//��ʹ���ڲ�����������
	GPIO_Init(GPIOA ,&GPIO_InitStructure);


	//��PA9��PA10�Ĺ��ܽ���ָ��Ϊ����1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	
	
	//���ô���1�Ĳ����������ʡ�����λ��У��λ��ֹͣλ��������
	USART_InitStructure.USART_BaudRate = baud;//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//��У��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�����ڷ��ͺͽ�������
	USART_Init(USART1, &USART_InitStructure);
	
	//ʹ�ܴ���1�Ľ����ж�
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//ʹ�ܴ���1����
	USART_Cmd(USART1, ENABLE);

}

int main(void )
{
	uint32_t d = 0x100;
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
	
	//��ʼ������1�Ĳ�����Ϊ115200bps
	//ע�⣺������յ����������룬Ҫ���PLL��
	usart1_init(115200);

	delay_ms(100);

	printf("Hello GEC\r\n");
	
	printf("d = %d\n", d);
	
	while(1)
	{
	
		
	}
	
	return 0 ;
}


void USART1_IRQHandler(void)
{

	uint8_t d;
	
	//����Ƿ���յ�����
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		d = USART_ReceiveData(USART1);
		
		if(d == 0x01)PFout(9)=0;
		if(d == 0xF1)PFout(9)=1;	
		if(d == 0x02)PFout(10)=0;
		if(d == 0xF2)PFout(10)=1;
		if(d == 0x03)PEout(13)=0;
		if(d == 0xF3)PEout(13)=1;
		if(d == 0x04)PEout(14)=0;
		if(d == 0xF4)PEout(14)=1;
		
		USART_SendData(USART1,d);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);		
		
		//��ձ�־λ������CPU��ǰ���ݽ�����ϣ����Խ����µ�����
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	
	}




}



