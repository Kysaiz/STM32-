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

void wwdg_init(void)
{
	//打开WWDG的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	
	//配置预分频值
	//窗口看门狗的硬件时钟频率=(42MHz/4096)/8=1281Hz
	WWDG_SetPrescaler(WWDG_Prescaler_8);

	
	//窗口的上限值
	WWDG_SetWindowValue(80);
	
	//配置它的计数值为127看，并使能窗口看门狗工作
	WWDG_Enable(127);

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

	
	
	//PG9输出端口为高阻态，这是引脚的初始电平状态
	PGout(9)=1;	
	
	
	//串口1初始化波特率为115200bps
	usart1_init(115200);
	
	delay_ms(500);
	
	//发送数据
	usart1_send_str("This is iwdg test\r\n");
	
	//检测是否由独立看门狗导致的复位
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET)
	{
	
		printf("iwdg reset cpu\r\n");
	
	}
	else if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) == SET)
	{
	
		printf("wwdg reset cpu\r\n");
	
	}
	else
	{
		printf("other reset cpu\r\n");		
	}
	
	//清空标志位
	RCC_ClearFlag();

	//窗口看门狗的初始化
	wwdg_init();
	while(1)
	{
		//延时40ms刚好位与时间窗口
		//delay_ms(40);

		//延时80ms，过晚了
		delay_ms(80);		
		//刷新计数值（将计数值恢复为127） == 喂狗操作
		WWDG_SetCounter(127);

		
		//delay_ms(3500);//这个已经超过了喂狗的时间，会触发系统复位。
		
		//等待串口1数据
		
		
		//OLED显示
		
		
		//获取温湿度
		
		
		//按键的检测
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
