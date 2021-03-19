#include "dac.h"

static DAC_InitTypeDef   		DAC_InitStructure;
static GPIO_InitTypeDef 		GPIO_InitStructure;

void dac_init(void)
{

	//�򿪶˿�A��Ӳ��ʱ�ӣ����ǶԶ˿�A����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	//����PA4����ΪANģʽ������ģ���ź�ģʽ
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
	
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//����Ҫ�������
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;//��DAC�����ѹ��ͨ��
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	
	//��ͨ��1
	DAC_Cmd(DAC_Channel_1, ENABLE);


}
