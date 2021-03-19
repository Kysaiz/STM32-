#include "beep.h"

static uint32_t tim13_cnt = 0;


void beep_pwm_init(uint32_t freq){
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  //使能(打开)端口F的硬件时钟，就是对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

	//初始化GPIO引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		//第13根引脚
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF;	  //复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推挽输出，增加输出电流能力。
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//高速响应
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//没有使能上下拉电阻

	GPIO_Init(GPIOF,&GPIO_InitStructure);
  /* Connect TIM13 pins to AF2 */  
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_TIM13);
  
   /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 10000/freq - 1;
  tim13_cnt = TIM_TimeBaseStructure.TIM_Period;
  TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;
  //  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
  
  
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 10000/freq - 1;                 //初始化时为100%占空比
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC1Init(TIM13, &TIM_OCInitStructure);
  
  /* TIM13 enable counter */
  TIM_Cmd(TIM13, ENABLE);
}


void tim13_set_freq(uint32_t freq){
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  /*定时器的基本配置，用于配置定时器的输出脉冲的频率为 freq Hz */
  TIM_TimeBaseStructure.TIM_Period = (10000/freq)-1; //设置定时脉冲的频率
  TIM_TimeBaseStructure.TIM_Prescaler = 8400-1; //第一次分频，简称为预分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  tim13_cnt = TIM_TimeBaseStructure.TIM_Period;
  TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
}


void tim13_set_duty(uint32_t duty)
{
  TIM_SetCompare1(TIM13,(tim13_cnt+1) * duty/100);
}



