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
	SysTick->CTRL = 0; // Disable SysTick，关闭系统定时器后才能取配置
	SysTick->LOAD = 21*n-1; 	// 填写计数值，就是我们的延时时间
	SysTick->VAL = 0; 			// 清空标志位
	SysTick->CTRL = 1; 			// 选中21MHz的时钟源，并开始让系统定时器工作
	while ((SysTick->CTRL & 0x10000)==0);//等待计数完毕
	SysTick->CTRL = 0; // 不再使用就关闭系统定时器
}


void delay_ms(uint32_t n)
{

	while(n--)
	{
		SysTick->CTRL = 0; // Disable SysTick，关闭系统定时器后才能取配置
		SysTick->LOAD = 21000-1; 	// 填写计数值，就是我们的延时时间
		SysTick->VAL = 0; 			// 清空标志位
		SysTick->CTRL = 1; 			// 选中21MHz的时钟源，并开始让系统定时器工作
		while ((SysTick->CTRL & 0x10000)==0);//等待计数完毕
		
	}
	SysTick->CTRL = 0; // 不再使用就关闭系统定时器
}

void usart1_init(uint32_t baud)
{
	//打开端口A硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE );
	
	//打开串口1的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );	
	
	//配置PA9和PA10引脚，为AF模式（复用功能模式）
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_10;//指定第9根引脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//配置为复用功能模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOA ,&GPIO_InitStructure);


	//将PA9和PA10的功能进行指定为串口1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	
	
	//配置串口1的参数：波特率、数据位、校验位、停止位、流控制
	USART_InitStructure.USART_BaudRate = baud;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//允许串口发送和接收数据
	USART_Init(USART1, &USART_InitStructure);
	
	
	//使能串口1的接收中断
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	
	
	//使能串口1工作
	USART_Cmd(USART1, ENABLE);

}



int32_t get_temp_humi(uint8_t *pbuf)
{
	uint32_t t = 0;
	int32_t i, j;
	uint8_t d = 0;
	uint8_t *p = pbuf;
	uint8_t check_sum = 0;
	
	// 配置PG9起始模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 设置为输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // 开漏输出模式
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStructure);	
	
	PGout(9)=0;
	delay_ms(18);
	PGout(9)=1;
	delay_us(30);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // 设置为输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	// PG9等待低电平出现，添加超时处理
	while (PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 4000) // 当t超过4ms时表示超时，退出函数，返回-1
		{
			return -1;
		}
	}
	
	// 检测低电平的持续时间
	t = 0;
	while (!PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 100) // 根据时序图，t应该为80us
		{
			return -2;
		}
	}
	
	// 检测高电平的持续时间
	t = 0;
	while (PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 100) // 根据时序图，t应该为80us
		{
			return -3;
		}
	}
	
	// 开始接收数据
	for (i = 0; i < 5; i++)
	{
		// 接收1个字节，高位优先接收数据
		d = 0;
		for (j = 7; j >= 0; j--)
		{
			// PG9等待高电平出现，添加超时处理
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
			
			// 延时40~60us
			delay_us(40);
			
			// 判断引脚的电平
			if (PGin(9))
			{
				//将d变量对应的bit位置1
				//0x01 1<<0:0000 0001
				//0x02 1<<1:0000 0010
				//.......
				//0x80 1<<7:1000 0000
				d|=1<<j;
				
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
		
		p[i] = d;
	}
	
	// 校验数据
	check_sum = (p[0] + p[1] + p[2] + p[3])&0xFF;
	
	// 如果校验不相符，返回-6
	if (check_sum != p[4])
		return -6;
			
	return 0;
	
}

int main(void )
{
	int32_t ret_val;
	uint8_t buf[5] = {0};
	
	//打开端口F的硬件时钟，等同于对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE );
	
	//使能端口G硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	//初始化
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	PFout(9) = 1;
	
	//配置GPIOF的第9根
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//初始化
	GPIO_Init(GPIOF,&GPIO_InitStructure);	
	
	
	//PG9输出高电平，这是引脚的初始电平状态
	PGout(9) = 1;	
	
	//初始化串口1的波特率为115200bps
	//注意：如果接收的数据是乱码，要检查PLL。
	usart1_init(115200);
	
	delay_ms(100);

	printf("This is dht11 test\r\n");
	
	while(1)
	{
		ret_val = get_temp_humi(buf);
		
		if (ret_val == 0)
		{
			printf("温度：%d.%d湿度：%d.%d\r\n", buf[2], buf[3], buf[0], buf[1]);
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
	
	
	//检测是否接收到数据
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		d = USART_ReceiveData(USART1);
		
		if(d == 'a')PFout(9)=0;
		if(d == 'A')PFout(9)=1;		
	
		
		//清空标志位，告诉CPU当前数据接收完毕，可以接收新的数据
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	
	}

}
