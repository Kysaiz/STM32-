#include "stm32f4xx.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_InitStructure ;
EXTI_InitTypeDef EXTI_InitStructure ;
NVIC_InitTypeDef NVIC_InitStructure ;
USART_InitTypeDef USART_InitStructure ;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint32_t cmp = 100;

#define PFout(n)	*((volatile uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PBout(n)	*((volatile uint32_t *)(0x42000000+(GPIOB_BASE+0x14-0x40000000)*32+(n)*4))
#define PEout(n)	*((volatile uint32_t *)(0x42000000+(GPIOE_BASE+0x14-0x40000000)*32+(n)*4))
#define PEin(n)		*((volatile uint32_t *)(0x42000000+(GPIOE_BASE+0x10-0x40000000)*32+(n)*4))

#pragma import(__use_no_semihosting_swi)


struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;


int fputc(int ch, FILE *f) 
{
		
	
	USART_SendData(USART1,ch);
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

void sr04_init(void)
{
	//ʹ�ܶ˿�B���˿�E��Ӳ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE );
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE );
	
	
	//PB6Ϊ���ģʽ����Ϊ���������ӵ�Trig
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//ָ����6������ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;//����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//�������ŵ���Ӧʱ��=1/100MHz .
	//�Ӹߵ�ƽ�л����͵�ƽ1/100MHz,�ٶ�Խ�죬���Ļ�Խ��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//��������ģʽ��������������͹����������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//��ʹ���ڲ�����������
	GPIO_Init(GPIOB ,&GPIO_InitStructure);	
	
	
	//PE6Ϊ����ģʽ����ΪҪ���ECHO����ߵ�ƽ�ĳ���ʱ��
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//ָ����6������ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;//����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//�������ŵ���Ӧʱ��=1/100MHz .
	//�Ӹߵ�ƽ�л����͵�ƽ1/100MHz,�ٶ�Խ�죬���Ļ�Խ��
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//��������ģʽ��������������͹����������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//��ʹ���ڲ�����������
	GPIO_Init(GPIOE ,&GPIO_InitStructure);	
	
	
	//PB6���ų�ʼ״̬Ϊ�͵�ƽ������ʱ��ͼ�����˽⵽
	PBout(6)=0;

}


uint32_t sr04_get_distance(void)
{
	uint32_t t=0;

	//PB6����ߵ�ƽ
	PBout(6)=1;
	
	//��ʱ10us
	delay_us(10);

	//PB6����͵�ƽ
	PBout(6)=0;	
	
	//PE6Ҫ�ȴ��ߵ�ƽ����
	while(PEin(6)==0);
	
	
	//�����ߵ�ƽ�ĳ���ʱ��
	while(PEin(6))
	{
	
		t++;
		delay_us(9);//������ÿ����9usʱ�䣬����Ϊ3mm
	
	}
	
	//��Ϊ��ʱ���ǰ�������ͷ��ص�ʱ�䣬��Ҫ����2
	t/=2;
	
	return 3*t;
}

void tim13_init(void)
{
	//��TIM13Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

	//����TIM13����ز���������ֵ��������ʱʱ�䣩����Ƶֵ
	//TIM13��Ӳ��ʱ��Ƶ��=84000000/8400=10000Hz
	//ֻҪ����10000�μ���������1��ʱ��ĵ��Ҳ����1Hz��Ƶ�����
	//ֻҪ����10000/100�μ���������1��/100ʱ��ĵ��Ҳ����100Hz��Ƶ�����
	TIM_TimeBaseStructure.TIM_Period = 10000/2-1;//����ֵ100-1�����������Ƶ��Ϊ100Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;//Ԥ��Ƶ��Ҳ���ǵ�һ�η�Ƶ����ǰ8400��Ƶ������84MHz/8400
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//ʱ�ӷ�Ƶ��Ҳ��֮���η�Ƶ���F407�ǲ�֧�ֵģ���˸ò����ǲ�����Ч
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���Ǵ�0��ʼ������Ȼ�������TIM_Period
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	
	//ռ�ձȵ�����
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//������PWM1ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�����������أ���ǰ���������
	TIM_OCInitStructure.TIM_Pulse = 0;								//���ֵ�Ǿ�����ռ�ձȣ����ñȽ�ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//��Ч��״̬Ϊ�ߵ�ƽ
	
	//TIM13��ͨ��1����
	TIM_OC1Init(TIM13, &TIM_OCInitStructure);
	
	//ʹ�ܶ�ʱ��13����
	TIM_Cmd(TIM13, ENABLE);
}


int main(void )
{
	uint32_t d=1000;
	//�򿪶˿�E��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�E����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE );
	
	//�򿪶˿�F��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�F����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE );
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	// �ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	//��ʼ����Ӧ�˿ڵ����� 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_10;// F9��F10��LED0��LED1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;//����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//�������ŵ���Ӧʱ��=1/100MHz .
	//�Ӹߵ�ƽ�л����͵�ƽ1/100MHz,�ٶ�Խ�죬���Ļ�Խ��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//��������ģʽ��������������͹����������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//��ʹ���ڲ�����������
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13|GPIO_Pin_14;// E13��E14��LED2��LED3
	GPIO_Init(GPIOE ,&GPIO_InitStructure);
	
	PFout(9)=PFout(10)=PEout(13)=PEout(14)=1;
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8; // ������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//����Ϊ����ģʽ
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
	// �жϽṹ������
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line2; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
	EXTI_Init(&EXTI_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line3; 
	EXTI_Init(&EXTI_InitStructure);	
	
	//��ʼ������1�Ĳ�����Ϊ115200bps
	//ע�⣺������յ����������룬Ҫ���PLL��
	usart1_init(115200);
	
	delay_ms(100);

	printf("�����״����\r\n");

	tim13_init();
	
	//��PF8���ӵ�TIM13
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_TIM13);

	sr04_init();
	
	while(1)
	{
		d = sr04_get_distance();
		
		if(d>=20 && d<=4000)
		{
			printf("distance = %d mm\r\n",d);
		
			if (d >= 200) //  20cm���밲ȫ��������LED��Ϩ��
			{
				PFout(9)=PFout(10)=PEout(13)=PEout(14)=1;
				TIM_SetCompare1(TIM13,0); // ����������
				delay_ms(940);
			}
			else if (d >= 150) // 15~20cm�ǳ���ȫ������1յLED��
			{
				PFout(9)=0;
				PFout(10)=PEout(13)=PEout(14)=1;
				TIM_SetCompare1(TIM13,0); // ����������
				delay_ms(940);
			}
			else if (d >= 100) // 10~15cm��ȫ������2յLED��
			{
				PFout(9)=PFout(10)=0;
				PEout(13)=PEout(14)=1;
				TIM_TimeBaseStructure.TIM_Period = 10000/2-1; // ���Ƶ��2Hz
				TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
				TIM_SetCompare1(TIM13,cmp);
				delay_ms(440);
			}
			else if (d >= 50) // 5~10cm����������3յLED��
			{
				PFout(9)=PFout(10)=PEout(13)=0;
				PEout(14)=1;
				TIM_TimeBaseStructure.TIM_Period = 10000/5-1; // ���Ƶ��5Hz
				TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
				TIM_SetCompare1(TIM13,cmp);
				delay_ms(440);
			}
			else // <5cm Σ��������4յLED��
			{
				PFout(9)=PFout(10)=PEout(13)=PEout(14)=0;
				TIM_TimeBaseStructure.TIM_Period = 10000/10-1; // ���Ƶ��10Hz
				TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
				TIM_SetCompare1(TIM13,cmp);
				delay_ms(440);
			}
		
		}
		
		delay_ms(60);
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
		
		if(d == 'a')PFout(9)=0;
		if(d == 'A')PFout(9)=1;		
	
		
		//��ձ�־λ������CPU��ǰ���ݽ�����ϣ����Խ����µ�����
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	
	}

}

void EXTI2_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		cmp = 100;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line2);		
	}

}

void EXTI3_IRQHandler(void)
{
	// ����Ƿ����жϴ���
	if (EXTI_GetITStatus(EXTI_Line3) == SET)
	{
		cmp = 400;
		// ��ձ�־λ������CPU����ǰ�Ѿ�����жϴ���������Ӧ�µ�һ���ж�����
		EXTI_ClearITPendingBit(EXTI_Line3);		
	}

}
