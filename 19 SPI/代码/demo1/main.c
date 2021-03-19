#include "stm32f4xx.h"
#include <stdio.h>

static GPIO_InitTypeDef 		GPIO_InitStructure;
static NVIC_InitTypeDef   		NVIC_InitStructure;
static USART_InitTypeDef   		USART_InitStructure;
static SPI_InitTypeDef  		SPI_InitStructure;

//使用到IDR寄存器
#define PAin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PEin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PGin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x10 - 0x40000000)*32 + n*4))

//使用到ODR寄存器
#define PEout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + n*4))
#define PBout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOB_BASE + 0x14 - 0x40000000)*32 + n*4))
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

void w25qxx_init(void)
{

	//使能端口B的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	//使能SPI1硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	
	//配置PB3 PB4 PB5为复用功能模式
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;	//第3 4 5个引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;			// 复用功能模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;		//引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		//增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;		//不需要上下拉电阻
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//指定PB3 PB4 PB5连接到SPI1硬件
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
	
	
	//PB14配置为输出模式
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;	//第14个引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;		//输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;		//引脚高速工作，收到指令立即工作；缺点：功耗高
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		//增加输出电流的能力
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;		//不需要上下拉电阻
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//PB14的初始电平状态？
	PBout(14) = 1;
	
	//SPI1的参数配置

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工的通信方式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//配置为主机角色，主动去控制时钟线
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//收发数据大小以字节为单位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//看从机数据手册，CPOL=1
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//看从机数据手册，CPHA=1。当前是使用模式3通信
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//芯片选择引脚通过软件代码控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//SPI硬件时钟=84MHz/16=5.25MHz,看从机数据手册
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//看从机数据手册，就是最高有效位优先传输
	//SPI_InitStructure.SPI_CRCPolynomial = 7;//用于两个M4芯片相互SPI通信
	SPI_Init(SPI1, &SPI_InitStructure);
	
	//使能SPI1工作
	SPI_Cmd(SPI1, ENABLE);

}

uint8_t SPI1_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty ,检测发送缓冲区是否为空*/
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral，通过SPI1发送字节 */
  SPI_I2S_SendData(SPI1, byte);

  /*!< Wait to receive a byte，等待接收字节完成 */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus ,返回接收到的数据*/
  return SPI_I2S_ReceiveData(SPI1);
}

void w25qxx_read_id(uint8_t *m_id,uint8_t *d_id)
{
	//片选引脚为低电平，从机开始工作
	PBout(14) = 0;
	
	//发送0x90
	SPI1_SendByte(0x90);
	
	//发送24bit地址，地址为0x000000
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);


	//读取厂商ID,传递参数任意
	*m_id = SPI1_SendByte(0xFF);
	
	
	
	//读取设备ID
	*d_id = SPI1_SendByte(0xFF);	
	
	
	//片选引脚为高电平，从机停止工作
	PBout(14) = 1;
	
	
}

void w25qxx_read_data(uint8_t *data)
{
	//片选引脚为低电平，从机开始工作
	PBout(14) = 0;
	
	//发送0x90
	SPI1_SendByte(0x03);
	
	//发送24bit地址，地址为0x000000
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);


	//读取厂商ID,传递参数任意
	*data = SPI1_SendByte(0xFF);	
	
	//片选引脚为高电平，从机停止工作
	PBout(14) = 1;
	
	
}


int main(void)
{
	uint8_t m_id,d_id;
	uint8_t data;
	uint16_t i;
	
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
	usart1_send_str("This is spi flash test\r\n");
	

	w25qxx_init();
	
	w25qxx_read_id(&m_id,&d_id);
	
	//准确的结果：m_id = EF   d_id=17
	printf("m_id=%02X d_id=%02X\r\n",m_id,d_id);
	
	
	
	//片选引脚为低电平，从机开始工作
	PBout(14) = 0;
	
	//发送0x03
	SPI1_SendByte(0x03);
	
	//发送24bit地址，地址为0x000000
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);


	// 读取64字节的数据
	for (i = 0; i < 64; i++)
	{
		data = SPI1_SendByte(0xFF);	
		printf("data[%d] = %x\r\n", i, data);
	}
	
	//片选引脚为高电平，从机停止工作
	PBout(14) = 1;
	
	printf("data receive over\r\n");

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


