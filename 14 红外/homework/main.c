#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

static GPIO_InitTypeDef 		GPIO_InitStructure;
static NVIC_InitTypeDef   		NVIC_InitStructure;
static USART_InitTypeDef   		USART_InitStructure;
static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
static TIM_OCInitTypeDef  TIM_OCInitStructure;

#define VOLUME 100

static volatile uint32_t tim14_cnt;
static volatile uint32_t tim13_cnt;

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

int32_t delay_us(uint32_t nus)
{
	uint32_t temp;

	SysTick->CTRL = 0; 					
	SysTick->LOAD = (nus*21)-1; 		
	SysTick->VAL = 0; 					
	SysTick->CTRL = 1; 					
	
	while(1)
	{
	
		temp=SysTick->CTRL;
		
		//检测count flag
		if(temp & 0x00010000)
			break;
		
		//检测系统定时器是否意外关闭	
		if((temp & 0x1)==0)
			return -1;		
	}
	
	SysTick->CTRL = 0; 					

	return 0;
}



int32_t  delay_ms(uint32_t nms)
{
	uint32_t t = nms;

	uint32_t temp;
	
	
	while(t--)
	{
		SysTick->CTRL = 0; 			
		SysTick->LOAD = 21000-1; 	
		SysTick->VAL = 0; 			
		SysTick->CTRL = 1; 			
		while(1)
		{

			temp=SysTick->CTRL;
			
			//检测count flag
			if(temp & 0x00010000)
				break;
			
			//检测系统定时器是否意外关闭	
			if((temp & 0x1)==0)
				return -1;		
		}
	}	
	
	SysTick->CTRL = 0; 	
	
	return 0;
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



int32_t HS0038_read_data(uint8_t *pbuf)
{
	uint32_t t=0;
	int32_t i=0,j=0;
	uint8_t d=0;
	uint8_t *p = pbuf;
	
	if (PAin(8))
		return -1;
	
	//通过超时检测低电平的合法性
	t=0;
	while(PAin(8)==0)
	{
		t++;
		delay_us(10);
		
		if(t>=1000)
		{
			//usart1_send_str("error code -2\r\n");
			return -2;
		}
	}


	//通过超时检测高电平的合法性
	t=0;
	while(PAin(8))
	{
		t++;
		delay_us(10);
		
		if(t>=500)
		{
			//usart1_send_str("error code -3\r\n");
			return -3;
		}
			
	}
	
	for(j=0; j<4; j++)
	{
		//接收一个字节，低位优先接收数据，bit7 bit6 ...... bit0
		d=0;
		for(i=0; i<8; i++)
		{
			//通过超时检测低电平的合法性
			t=0;
			while(PAin(8)==0)
			{
				t++;
				delay_us(10);
				
				if(t>=100)
				{
					//usart1_send_str("error code -4\r\n");
					return -4;
				}
					
			}
		
			//延时600us~900us左右
			delay_us(600);
			
			//判断引脚的电平
			if(PAin(8))
			{
				//将d变量对应的bit位置1
				//0x01 1<<0:0000 0001
				//0x02 1<<1:0000 0010
				//.......
				//0x80 1<<7:1000 0000
				d|=1<<i;
				
				//等待剩下的高电平持续完毕
				t=0;
				while(PAin(8))
				{
					t++;
					delay_us(10);
					
					if(t>=200)
					{
						//usart1_send_str("error code -5\r\n");
						return -5;
					}
				}			
			
			}
		
		}	
	
		p[j]=d;
	}
	
	//校验数据
	if ((p[0] + p[1]) == 255)
		if ((p[2] + p[3]) == 255)
			return 0;
		
	return -6;
}

void tim14_init(void)
{
	//打开TIM14的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	
	//配置TIM14的相关参数：计数值（决定定时时间）、分频值
	//TIM14的硬件时钟频率=84000000/8400=10000Hz
	//只要进行10000次计数，就是1秒时间的到达，也就是1Hz的频率输出
	//只要进行10000/100次计数，就是1秒/100时间的到达，也就是100Hz的频率输出
	TIM_TimeBaseStructure.TIM_Period = 10000/1000-1;//计数值100-1，决定了输出频率为100Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;//预分频，也就是第一次分频，当前8400分频，就是84MHz/8400
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//时钟分频，也称之二次分频这个F407是不支持的，因此该参数是不会生效
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//就是从0开始计数，然后计数到TIM_Period
	tim14_cnt = TIM_TimeBaseStructure.TIM_Period+1;
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
	
	//占空比的配置
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//工作在PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//脉冲的输出开关，当前是输出脉冲
	TIM_OCInitStructure.TIM_Pulse = tim14_cnt;		//这个值是决定了占空比，配置比较值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//有效的状态为高电平
	
	//TIM14的通道1配置
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
	
	//使能定时器14工作
	TIM_Cmd(TIM14, ENABLE);
}

void tim14_set_freq(uint32_t freq)
{
    /*定时器的基本配置，用于配置定时器的输出脉冲的频率为 freq Hz */
    TIM_TimeBaseStructure.TIM_Period = (10000/freq)-1; //设置定时脉冲的频率
    TIM_TimeBaseStructure.TIM_Prescaler = 8400-1; //第一次分频，简称为预分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    tim14_cnt= TIM_TimeBaseStructure.TIM_Period;
    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
}

void tim14_set_duty(uint32_t duty)
{
	TIM_SetCompare1(TIM14,(tim14_cnt+1) * duty/100);
}

void tim13_init(void)
{
	//打开TIM13的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

	
	//配置TIM13的相关参数：计数值（决定定时时间）、分频值
	//TIM13的硬件时钟频率=84000000/8400=10000Hz
	//只要进行10000次计数，就是1秒时间的到达，也就是1Hz的频率输出
	//只要进行10000/100次计数，就是1秒/100时间的到达，也就是100Hz的频率输出
	TIM_TimeBaseStructure.TIM_Period = 10000/10-1;//计数值100-1，决定了输出频率为100Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;//预分频，也就是第一次分频，当前8400分频，就是84MHz/8400
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//时钟分频，也称之二次分频这个F407是不支持的，因此该参数是不会生效
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//就是从0开始计数，然后计数到TIM_Period
	tim13_cnt= TIM_TimeBaseStructure.TIM_Period;
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	
	//占空比的配置
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//工作在PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//脉冲的输出开关，当前是输出脉冲
	TIM_OCInitStructure.TIM_Pulse = tim13_cnt;		//这个值是决定了占空比，配置比较值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//有效的状态为高电平
	
	//TIM13的通道1配置
	TIM_OC1Init(TIM13, &TIM_OCInitStructure);
	
	//使能定时器13工作
	TIM_Cmd(TIM13, ENABLE);
}

void tim13_set_freq(uint32_t freq)
{
    /*定时器的基本配置，用于配置定时器的输出脉冲的频率为 freq Hz */
    TIM_TimeBaseStructure.TIM_Period = (10000/freq)-1; //设置定时脉冲的频率
    TIM_TimeBaseStructure.TIM_Prescaler = 8400-1; //第一次分频，简称为预分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    tim13_cnt= TIM_TimeBaseStructure.TIM_Period;
    TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
}

void tim13_set_duty(uint32_t duty)
{
	TIM_SetCompare1(TIM13,(tim13_cnt+1) * duty * VOLUME /10000);
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

void dht11_init()
{
	//使能端口G硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//初始化
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	//PG9输出高电平，这是引脚的初始电平状态
	PGout(9)=1;	
}

int main(void)
{
	uint8_t buf[5]={0};  // 红外
	uint8_t buf2[5]={0}; // 温湿度
	int32_t rt=0;		 // 红外rt
	int32_t rt_dht11=0;  // 温湿度rt
	
	int32_t duty = 100;
	uint32_t beep_freq = 100;
	int32_t beep_duty = 40;
	
	//打开端口F的硬件时钟，就是对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	//配置GPIOF的第9根
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//初始化
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	PEout(13)=PEout(14)=PFout(10)=1;
	
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);
	
	tim14_init();
	
	//初始化对应端口的引脚 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;//指定第8根引脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//配置为复用模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
	//将PF8连接到TIM13
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_TIM13);
	
	tim13_init();
	tim13_set_duty(0);
	
	//使能端口A硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//初始化
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//串口1初始化波特率为115200bps
	usart1_init(115200);
	
	delay_ms(100);
	
	//发送数据
	usart1_send_str("This is HS0038 test\r\n");
	
	while(1)
	{
		rt = HS0038_read_data(buf);
		
		if(rt == 0)
		{
			printf("ir data: %02X %02X %02X %02X \r\n",buf[0],buf[1],buf[2],buf[3]);
			
			if (buf[2] == 0x07) // 按键 "-" 控制LED1变暗
			{
				if (duty + 20 <= 100)
				{
					duty += 20;
				}
				
				printf("LED DOWN\r\n");
				tim14_set_duty(duty);
			}
			else if (buf[2] == 0x15) // 按键 "+" 控制LED1变亮
			{
				if (duty - 20 > 0)
				{
					duty -= 20;
				}
				
				printf("LED UP\r\n");
				tim14_set_duty(duty);
			}
			else if (buf[2] == 0x0C) // 按键 "1" 控制LED1灯亮灭
			{
				if (duty != 100)
				{
					duty = 100;
				}
				else 
				{
					duty = 5;
				}
				
				printf("LED1 \r\n");
				tim14_set_duty(duty);
			}
			else if (buf[2] == 0x18) // 按键 "2" 控制LED2灯亮灭
			{
				PFout(10)^=1;
				printf("LED2 \r\n");
			}
			else if (buf[2] == 0x5E) // 按键 "3" 控制LED3灯亮灭
			{
				PEout(13)^=1;
				printf("LED3 \r\n");
			}
			else if (buf[2] == 0x08) // 按键 "4" 控制LED4灯亮灭
			{
				PEout(14)^=1;
				printf("LED4 \r\n");
			}
			else if (buf[2] == 0x1C) // 按键 "5" 控制蜂鸣器频率加100hz（上限为1000）
			{
				if (beep_freq + 100 <= 1000)
				{
					beep_freq += 100;
				}
				
				printf("BEEP freq +100, now %d\r\n", beep_freq);
				tim13_set_freq(beep_freq);
				tim13_set_duty(beep_duty);
			}
			else if (buf[2] == 0x5A) // 按键 "6" 控制蜂鸣器频率减100hz（下限为1）
			{
				if (beep_freq - 100 >= 1)
				{
					beep_freq -= 100;
				}
				else
				{
					beep_freq = 1;
				}
				
				printf("BEEP freq -100, now %d\r\n", beep_freq);
				tim13_set_freq(beep_freq);
				tim13_set_duty(beep_duty);
			}
			else if (buf[2] == 0x42) // 按键 "7" 控制蜂鸣器音量变大，占空比加20
			{
				if (beep_duty + 20 < 100)
				{
					beep_duty += 20;
				}
				
				printf("VOLUME UP, now duty = %d\r\n", beep_duty);
				tim13_set_duty(beep_duty);
			}
			else if (buf[2] == 0x52) // 按键 "8" 控制蜂鸣器音量变小，占空比减20
			{
				if (beep_duty - 20 >= 0)
				{
					beep_duty -= 20;
				}
				
				printf("VOLUME DOWN, now duty = %d\r\n", beep_duty);
				tim13_set_duty(beep_duty);
			}
			else if (buf[2] == 0x46) // 按键 "CH" 温湿度读取
			{
				dht11_init();
				
				rt_dht11 = dht11_read_data(buf2);
				if(rt_dht11 == 0)
				{
					printf("T:%d.%d,H:%d.%d\r\n",buf2[2],buf2[3],buf2[0],buf2[1]);
				
				}
				else
				{
					printf("dht11 error code = %d\r\n", rt_dht11);
				
				}
			}
		
		}
		
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
