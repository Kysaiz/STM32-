#include "stm32f4xx.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_InitStructure ;
EXTI_InitTypeDef EXTI_InitStructure ;
NVIC_InitTypeDef NVIC_InitStructure ;
USART_InitTypeDef USART_InitStructure ;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

#define PFout(n)	*((volatile uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PBout(n)	*((volatile uint32_t *)(0x42000000+(GPIOB_BASE+0x14-0x40000000)*32+(n)*4))
#define PEout(n)	*((volatile uint32_t *)(0x42000000+(GPIOE_BASE+0x14-0x40000000)*32+(n)*4))
#define PEin(n)		*((volatile uint32_t *)(0x42000000+(GPIOE_BASE+0x10-0x40000000)*32+(n)*4))
	
#define PGout(n)	*((volatile uint32_t *)(0x42000000+(GPIOG_BASE+0x14-0x40000000)*32+(n)*4))
#define PGin(n)		*((volatile uint32_t *)(0x42000000+(GPIOG_BASE+0x10-0x40000000)*32+(n)*4))

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



int32_t get_temp_humi(uint8_t *pbuf)
{
	uint32_t t = 0;
	int32_t i, j;
	uint8_t d = 0;
	uint8_t *p = pbuf;
	uint8_t check_sum = 0;
	
	// ����PG9��ʼģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // ����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // ��©���ģʽ
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStructure);	
	
	PGout(9)=0;
	delay_ms(18);
	PGout(9)=1;
	delay_us(30);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // ����Ϊ����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	// PG9�ȴ��͵�ƽ���֣���ӳ�ʱ����
	while (PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 4000) // ��t����4msʱ��ʾ��ʱ���˳�����������-1
		{
			return -1;
		}
	}
	
	// ���͵�ƽ�ĳ���ʱ��
	t = 0;
	while (!PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 100) // ����ʱ��ͼ��tӦ��Ϊ80us
		{
			return -2;
		}
	}
	
	// ���ߵ�ƽ�ĳ���ʱ��
	t = 0;
	while (PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 100) // ����ʱ��ͼ��tӦ��Ϊ80us
		{
			return -3;
		}
	}
	
	// ��ʼ��������
	for (i = 0; i < 5; i++)
	{
		// ����1���ֽڣ���λ���Ƚ�������
		d = 0;
		for (j = 7; j >= 0; j--)
		{
			// PG9�ȴ��ߵ�ƽ���֣���ӳ�ʱ����
			t = 0;
			while (!PGin(9))
			{
				t++;
				delay_us(1);
				
				if (t >= 100)
				{
					return -4;
				}
			}
			
			// ��ʱ40~60us
			delay_us(40);
			
			// �ж����ŵĵ�ƽ
			if (PGin(9))
			{
				//��d������Ӧ��bitλ��1
				//0x01 1<<0:0000 0001
				//0x02 1<<1:0000 0010
				//.......
				//0x80 1<<7:1000 0000
				d|=1<<j;
				
				//�ȴ�ʣ�µĸߵ�ƽ�������
				t=0;
				while(PGin(9))
				{
					t++;
					delay_us(1);
					
					if(t>=100)
						return -5;
				}	
			}
		}
		
		p[i] = d;
	}
	
	// У������
	check_sum = (p[0] + p[1] + p[2] + p[3])&0xFF;
	
	// ���У�鲻���������-6
	if (check_sum != p[4])
		return -6;
			
	return 0;
	
}

int main(void )
{
	int32_t ret_val;
	uint8_t buf[5] = {0};
	
	//�򿪶˿�F��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�F����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE );
	
	//ʹ�ܶ˿�GӲ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	//��ʼ��
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	PFout(9) = 1;
	
	//����GPIOF�ĵ�9��
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//��ʼ��
	GPIO_Init(GPIOF,&GPIO_InitStructure);	
	
	
	//PG9����ߵ�ƽ���������ŵĳ�ʼ��ƽ״̬
	PGout(9) = 1;	
	
	//��ʼ������1�Ĳ�����Ϊ115200bps
	//ע�⣺������յ����������룬Ҫ���PLL��
	usart1_init(115200);
	
	delay_ms(100);

	printf("This is dht11 test\r\n");
	
	while(1)
	{
		ret_val = get_temp_humi(buf);
		
		if (ret_val == 0)
		{
			printf("�¶ȣ�%d.%dʪ�ȣ�%d.%d\r\n", buf[2], buf[3], buf[0], buf[1]);
		}
		else
		{
			printf("dht11 error code = %d\r\n", ret_val);
		}
		
		delay_ms(5000);
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
