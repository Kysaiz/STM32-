#include "stm32f4xx.h"
#include <stdio.h>

static GPIO_InitTypeDef 		GPIO_InitStructure;
static NVIC_InitTypeDef   		NVIC_InitStructure;
static USART_InitTypeDef   		USART_InitStructure;


//使用到IDR寄存器
#define PAin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PBin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOB_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PEin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PGin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x10 - 0x40000000)*32 + n*4))

//使用到ODR寄存器
#define PBout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOB_BASE + 0x14 - 0x40000000)*32 + n*4))
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


#define SCL_W		PBout(8)
#define SDA_W		PBout(9)
#define SDA_R		PBin(9)



void i2c_init(void)
{
	//端口B硬件时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	//PB8 PB9为输出模式
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//PB8和PB9初始的电平状态，通过时序图看到，都是高电平
	PBout(8)=PBout(9)=1;

}

void sda_pin_mode(GPIOMode_TypeDef pin_mode)
{
	//配置GPIOB的第9根
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = pin_mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//初始化
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}


void i2c_start(void)
{
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);

	SCL_W=1;
	SDA_W=1;
	delay_us(5);
	
	SDA_W=0;
	delay_us(5);
	
	SCL_W=0;
	delay_us(5);
}

void i2c_stop(void)
{
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);

	SCL_W=1;
	SDA_W=0;
	delay_us(5);
	
	SDA_W=1;
	delay_us(5);
}


void i2c_send_byte(uint8_t txd)
{
	int32_t i;
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);
	
	SCL_W=0;
	SDA_W=0;
	delay_us(5);
		
	for(i=7; i>=0; i--)
	{
		if(txd & (1<<i))//bit7 -> bit0
			SDA_W=1;
		else
			SDA_W=0;
		
		delay_us(5);	
	
		SCL_W=1;
		delay_us(5);
		
		SCL_W=0;
		delay_us(5);
	}
}

void i2c_recv_byte(void)
{
	int32_t i;
	uint8_t byte = 0;
	
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);
	
	SCL_W = 0;
	SDA_W = 0;
	
	delay_us(5);
	
	//保证SDA引脚为输入模式
	sda_pin_mode(GPIO_Mode_IN);
		
	for(i=7; i>=0; i--)
	{
		//设置时钟为高电平，告诉从机，可以对SDA引脚进行控制
		SCL_W = 1;
		delay_us(5);
		
		//读取SDA引脚的电平
		if (SDA_R)
			byte|= 1<<i;
		
		//设置时钟为低电平，告诉从机，不可以对SDA引脚进行控制
		SCL_W=0;
		delay_us(5);  
	}
}


uint8_t i2c_ack(void)
{
	uint8_t ack;
	//保证SDA引脚为输入模式
	sda_pin_mode(GPIO_Mode_IN);

	SCL_W=1;
	delay_us(5);
	
	//读取SDA引脚的电平
	if(SDA_R)
		ack=1;
	else
		ack=0;
	
	
	return ack;

}


int32_t at24c02_write(uint8_t addr,uint8_t *buf,uint8_t len)
{
	uint8_t *p = buf;
	
	//发送起始信号
	i2c_start();
	
	//设备寻址写访问，地址为0xA0
	i2c_send_byte(0xA0);
	
	//等待应答信号
	if(i2c_ack())
	{
		printf("device address fail\r\n");
		return -1;
	}

	//告诉AT24C02，要写入数据的地址
	i2c_send_byte(addr);
	
	//等待应答信号
	if(i2c_ack())
	{
		printf("word address fail\r\n");
		return -2;
	}

	//连续的写入数据
	while(len--)
	{
		i2c_send_byte(*p++);
		
		
		if(i2c_ack())
		{
			printf("write data fail\r\n");
			return -3;
		}	
	}

	//发送停止信号
	i2c_stop();
	
	printf("write success\r\n");
	
	return 0;

}

int32_t at24c02_read(uint8_t addr,uint8_t *buf,uint8_t len)
{
	uint8_t *p = buf;
	
	//发送起始信号
	i2c_start();
	
	//设备寻址写访问，地址为0xA0
	i2c_send_byte(0xA0);
	
	//等待应答信号
	if(i2c_ack())
	{
		printf("device address fail\r\n");
		return -1;
	}

	//告诉AT24C02，要读取数据的地址
	i2c_send_byte(addr);
	
	//等待应答信号
	if(i2c_ack())
	{
		printf("word address fail\r\n");
		return -2;
	}

	//再一次发送起始信号
	i2c_start();
	
	//发送寻址地址，读访问操作
	i2c_send_byte(0xA1);
	
	//等待应答信号
	if(i2c_ack())
	{
		printf("device address_r fail\r\n");
		return -3;
	}
	
	//连续的读取数据
	while(len--)
	{
		i2c_send_byte(*p++);
		
		
		if(i2c_ack())
		{
			printf("read data fail\r\n");
			return -3;
		}	
	}

	//发送停止信号
	i2c_stop();
	
	printf("read success\r\n");
	
	return 0;

}

int main(void)
{
	uint32_t i=0;
	int32_t rt=0;
	
	uint8_t buf1[8]={1,2,3,4,5,6,7,8};
	uint8_t buf2[8];

	
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
	usart1_send_str("This is i2c  test\r\n");
	
	
	i2c_init();
	
	
	at24c02_write(0,buf1,8);

	delay_ms(5);
	
	at24c02_read(0,buf2,8);

	for (i = 0; i < 8; i++)
	{
		printf("buf[%d] = %d\r\n", i, buf2[i]);
		
	}

	while(1)
	{

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


