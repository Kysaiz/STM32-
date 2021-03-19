#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#define PFout(n)	*((uint32_t *)(0x42000000+((uint32_t)(&GPIOF->ODR)-0x40000000)*32+(n)*4))
//#define PFout(n)	*((uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PFout(n)	*((volatile uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PBout(n)	*((volatile uint32_t *)(0x42000000+(GPIOB_BASE+0x14-0x40000000)*32+(n)*4))
#define PEin(n)		*((volatile uint32_t *)(0x42000000+(GPIOE_BASE+0x10-0x40000000)*32+(n)*4))

#pragma import(__use_no_semihosting_swi)

GPIO_InitTypeDef 	GPIO_InitStructure ;
EXTI_InitTypeDef 	EXTI_InitStructure ;
NVIC_InitTypeDef 	NVIC_InitStructure ;
USART_InitTypeDef 	USART_InitStructure ;
//TIM_InitTypeDef 	TIM_InitStructure ;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

static volatile uint32_t g_usart3_event=0;		//ֻҪ���ݰ�������ϣ��ñ����͵���1
static volatile uint8_t  g_usart3_buf[64]={0};	//���ݻ�����
static volatile uint32_t g_usart3_cnt=0;		//������Ŀ�ļ���

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



void usart3_init(uint32_t baud)
{
	//�򿪶˿�BӲ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE );
	
	//�򿪴���3��Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE );	
	
	//����PB10��PB11���ţ�ΪAFģʽ�����ù���ģʽ��
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11;//ָ����10 11������ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//����Ϊ���ù���ģʽ
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//�������ŵ���Ӧʱ��=1/100MHz .
	//�Ӹߵ�ƽ�л����͵�ƽ1/100MHz,�ٶ�Խ�죬���Ļ�Խ��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//��������ģʽ��������������͹����������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//��ʹ���ڲ�����������
	GPIO_Init(GPIOB ,&GPIO_InitStructure);


	//��PB10��PB11�Ĺ��ܽ���ָ��Ϊ����1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);	
	
	//���ô���1�Ĳ����������ʡ�����λ��У��λ��ֹͣλ��������
	USART_InitStructure.USART_BaudRate = baud;//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//��У��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�����ڷ��ͺͽ�������
	USART_Init(USART3, &USART_InitStructure);
	
	
	//ʹ�ܴ���3�Ľ����ж�
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	
	
	//ʹ�ܴ���3����
	USART_Cmd(USART3, ENABLE);

}


void usart3_send_str(char *str)
{
	char *p = str;
	
	while(p && (*p!='\0'))
	{
	
		USART_SendData(USART3,*p);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);

		p++;
	
	}


}

void tim14_init(void)
{
	//��TIM14��Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	
	//����TIM14����ز���������ֵ��������ʱʱ�䣩����Ƶֵ
	//TIM14��Ӳ��ʱ��Ƶ��=84000000/8400=10000Hz
	//ֻҪ����10000�μ���������1��ʱ��ĵ��Ҳ����1Hz��Ƶ�����
	//ֻҪ����10000/100�μ���������1��/100ʱ��ĵ��Ҳ����100Hz��Ƶ�����
	TIM_TimeBaseStructure.TIM_Period = 10000/100-1;//����ֵ100-1�����������Ƶ��Ϊ100Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;//Ԥ��Ƶ��Ҳ���ǵ�һ�η�Ƶ����ǰ8400��Ƶ������84MHz/8400
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//ʱ�ӷ�Ƶ��Ҳ��֮���η�Ƶ���F407�ǲ�֧�ֵģ���˸ò����ǲ�����Ч
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���Ǵ�0��ʼ������Ȼ�������TIM_Period
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
	
	//ռ�ձȵ�����
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//������PWM1ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�����������أ���ǰ���������
	TIM_OCInitStructure.TIM_Pulse = 100;							//���ֵ�Ǿ�����ռ�ձȣ����ñȽ�ֵ��Ĭ��LED��ΪϨ��״̬
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//��Ч��״̬Ϊ�ߵ�ƽ
	
	//TIM14��ͨ��1����
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
	

	//ʹ�ܶ�ʱ��14����
	TIM_Cmd(TIM14, ENABLE);
}

int main(void )
{
	
	char *p=NULL;
	
	uint32_t duty=0;
	
	char buf[64]={0};
	
	//�򿪶˿�E��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�E����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE );
	
	//�򿪶˿�F��Ӳ��ʱ�ӣ���ͬ�ڶԶ˿�F����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE );
	
	//��ʼ����Ӧ�˿ڵ����� 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;//ָ����9������ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//����Ϊ����ģʽ
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//�������ŵ���Ӧʱ��=1/100MHz .
	//�Ӹߵ�ƽ�л����͵�ƽ1/100MHz,�ٶ�Խ�죬���Ļ�Խ��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//��������ģʽ��������������͹����������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//��ʹ���ڲ�����������
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
   
   	/* ����PF8 beep����Ϊģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;					//��8������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//���ø���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//����Ҫ��������
	GPIO_Init(GPIOF, &GPIO_InitStructure);	 
    
    PFout(8) =0 ;
    
	//��PF9���ӵ�TIM14
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);
		
	tim14_init();
	//��ʼ������1�Ĳ�����Ϊ115200bps
	//ע�⣺������յ����������룬Ҫ���PLL��
	usart1_init(115200);
	delay_ms(100);
	//����3�Ĳ�����Ϊ9600bps����Ϊ����ģ��Ĭ��ʹ�øò�����
	usart3_init(9600);
	
	delay_ms(500);

	printf("This is Bluetooth test\r\n");
	
	


	while(1)
	{
		//����Ƿ���յ����ݰ�
		if(g_usart3_event)
		{
			//"duty=50#"
			if(strstr((const char *)g_usart3_buf,"duty"))
			{
				p = strtok((char *)g_usart3_buf,"=");// p="duty"
				printf("strtok[1]=%s\r\n",p);
				
				
				p=strtok(NULL,"=");
				printf("strtok[2]=%s\r\n",p);//p="50#"	

				duty = atoi(p);			//atoi("50#") -> 50
				
				//ͨ��ռ�ձ����õƹ������
				TIM_SetCompare1(TIM14,duty);
				
				sprintf(buf,"duty set %d ok\r\n",duty);
				
				usart3_send_str(buf);
			
			}
            //"beep=on#"
			if(strstr((const char *)g_usart3_buf,"beep=on"))
			{

                PFout(8) =1 ;
                printf("beep on ok\r\n");
				
				usart3_send_str("beep on ok\r\n");
			
			}
            
            //"beep=off#"
			if(strstr((const char *)g_usart3_buf,"beep=off"))
			{

                PFout(8) =0 ;
                printf("beep off ok\r\n");
				
				usart3_send_str("beep off ok\r\n");
			
			}
            
            
            

			memset((void *)g_usart3_buf,0,sizeof g_usart3_buf);
			g_usart3_cnt=0;
			g_usart3_event=0;
		}
		
	
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
		
			
		//��ձ�־λ������CPU��ǰ���ݽ�����ϣ����Խ����µ�����
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	
	}
}

void USART3_IRQHandler(void)
{

	uint8_t d;
	
	
	//����Ƿ���յ�����
	if(USART_GetITStatus(USART3,USART_IT_RXNE)==SET)
	{
		d = USART_ReceiveData(USART3);
		
		g_usart3_buf[g_usart3_cnt]=d;

		g_usart3_cnt++;
		
		//����Ƿ����ݽ������
		//������ݵļ����Ƿ���ڻ�����
		if((d == '#') || (g_usart3_cnt>sizeof g_usart3_buf))
		{
			g_usart3_event=1;
		
		}
		
		//��ձ�־λ������CPU��ǰ���ݽ�����ϣ����Խ����µ�����
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	
	}
}

