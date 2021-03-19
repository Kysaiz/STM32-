#include "stm32f4xx.h"
#include <stdio.h>

static GPIO_InitTypeDef 		GPIO_InitStructure;
static NVIC_InitTypeDef   		NVIC_InitStructure;
static USART_InitTypeDef   		USART_InitStructure;


//使用到IDR寄存器
#define PAin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PEin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PGin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x10 - 0x40000000)*32 + n*4))

//使用到ODR寄存器
#define PEout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + n*4))
#define PFout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000)*32 + n*4))
#define PGout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x14 - 0x40000000)*32 + n*4))	

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) 
{
	USART_SendData(USART1,ch);
		
	//等待数据发送成功
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);


	return ch;
}

void _sys_exit(int return_code) {

}

void delay_us(uint32_t n)
{
	SysTick->CTRL = 0; // Disable SysTick
	SysTick->LOAD = n*168-1; // 计数值 
	SysTick->VAL  = 0; // Clear current value as well as count flag
	SysTick->CTRL = 5; // Enable SysTick timer with processor clock(168MHz)
	while ((SysTick->CTRL & 0x00010000)==0);// Wait until count flag is set
	SysTick->CTRL = 0; // Disable SysTick	
}



void delay_ms(uint32_t n)
{
	while(n--)
	{
		SysTick->CTRL = 0; // Disable SysTick
		SysTick->LOAD = 168000-1; // 计数值 Count from n to 0 (168000-1+1=168000 cycles)
		SysTick->VAL = 0; // Clear current value as well as count flag
		SysTick->CTRL = 5; // Enable SysTick timer with processor clock(168MHz)
		while ((SysTick->CTRL & 0x00010000)==0);// Wait until count flag is set
		SysTick->CTRL = 0; // Disable SysTick	
	
	}
}


void usart1_init(uint32_t baud)
{
	//使能端口A硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	//使能串口1硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	
	//配置PA9、PA10为复用功能引脚
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//将PA9、PA10连接到USART1的硬件
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	
	//配置USART1的相关参数：波特率、数据位、校验位
	USART_InitStructure.USART_BaudRate = baud;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//允许串口发送和接收数据
	USART_Init(USART1, &USART_InitStructure);
	
	
	//使能串口接收到数据触发中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//使能串口1工作
	USART_Cmd(USART1,ENABLE);
}

void usart1_send_str(char *str)
{

	char *p = str;
	
	while(*p!='\0')
	{
		USART_SendData(USART1,*p);
		
		p++;
		
		
		//等待数据发送成功
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	
	}
}



int32_t dht11_read_data(uint8_t *pbuf)
{
	uint32_t t=0;
	int32_t i=0,j=0;
	uint8_t d=0;
	uint8_t *p = pbuf;
	uint8_t check_sum=0;
	
	//保证PG9为输出模式
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	//PG9输出低电平
	PGout(9)=0;
	
	
	//延时至少18ms
	delay_ms(18);
	
	//PG9输出高电平
	PGout(9)=1;	
	
	//延时30us
	delay_us(30);
	
	//保证PG9为输入模式
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG,&GPIO_InitStructure);

	//等待低电平出现，添加超时处理
	t=0;
	while(PGin(9))
	{
		t++;
		
		delay_us(1);
		
		if(t>=4000)
			return -1;
	}
	
	//通过超时检测低电平的合法性
	t=0;
	while(PGin(9)==0)
	{
		t++;
		delay_us(1);
		
		if(t>=100)
			return -2;
	}


	//通过超时检测高电平的合法性
	t=0;
	while(PGin(9))
	{
		t++;
		delay_us(1);
		
		if(t>=100)
			return -3;
	}
	
	for(j=0; j<5; j++)
	{
		//接收一个字节，高位优先接收数据，bit7 bit6 ...... bit0
		d=0;
		for(i=7; i>=0; i--)
		{
			//通过超时检测低电平的合法性
			t=0;
			while(PGin(9)==0)
			{
				t++;
				delay_us(1);
				
				if(t>=100)
					return -4;
			}
		
			//延时40us~60us左右
			delay_us(40);
			
			//判断引脚的电平
			if(PGin(9))
			{
				//将d变量对应的bit位置1
				//0x01 1<<0:0000 0001
				//0x02 1<<1:0000 0010
				//.......
				//0x80 1<<7:1000 0000
				d|=1<<i;
				
				//等待剩下的高电平持续完毕
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
	
		p[j]=d;
	}
	
	//校验数据
	check_sum = (p[0]+p[1]+p[2]+p[3])&0xFF;
	
	if(check_sum != p[4])
		return -6;
	
	return 0;
}



int main(void)
{
	uint8_t buf[5]={0};
	int32_t rt=0;
	
	//打开端口F的硬件时钟，就是对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

	
	//配置GPIOF的第9根
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//初始化
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	
	PFout(9)=1;
	
	//使能端口G硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);


	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//初始化
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	
	//PG9输出高电平，这是引脚的初始电平状态
	PGout(9)=1;	
	
	
	//串口1初始化波特率为115200bps
	usart1_init(115200);
	
	
	//发送数据
	usart1_send_str("This is dht11 test\r\n");
	
	
	
	while(1)
	{
		rt = dht11_read_data(buf);
		
		if(rt == 0)
		{
			printf("T:%d.%d,H:%d.%d\r\n",buf[2],buf[3],buf[0],buf[1]);
		
		}
		else
		{
			printf("dht11 error code = %d\r\n",rt);
		
		}
		
		
		delay_ms(6000);
	}

}

void USART1_IRQHandler(void)
{
	static uint8_t d=0;
	
	//检测是否接收到数据
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		d=USART_ReceiveData(USART1);
		
		//将接收到的数据返发给PC
		USART_SendData(USART1,d);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
		
		
		//0x01，点亮PF9
		if(d == 0x01)PFout(9)=0;
		
		//0xF1，熄灭PF9
		if(d == 0xF1)PFout(9)=1;
		
		
		//清空标志位，可以响应新的中断请求
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	
	
	




}
