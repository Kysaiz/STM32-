#include "adc.h"

static GPIO_InitTypeDef 		GPIO_InitStructure;
static ADC_InitTypeDef   		ADC_InitStructure;
static ADC_CommonInitTypeDef 	ADC_CommonInitStructure;

void adc_init(void)
{
	//�򿪶˿�A��Ӳ��ʱ�ӣ����ǶԶ˿�A����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	//��ADC1��Ӳ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	//����PA5����ΪANģʽ������ģ���ź�ģʽ
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
	//����ADC1��صĲ���
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ADC����
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//ADC123��Ӳ��ʱ��Ƶ��=84MHz/2=42MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//Ĭ�ϲ�ʹ��dma
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//����������ļ��=5*(1/42MHz)
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//ʹ��ADC1����
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//ʹ����߾��Ƚ��в���

	
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//ֻɨ��һ��ͨ����ENABLE����ɨ����ͨ����
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//���ϵĶ�ͨ����ģ���źŽ���ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//����Ҫ����������ADC����
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���洢
	ADC_InitStructure.ADC_NbrOfConversion = 1;//ֻ��1��ͨ����Ҫ����ת��
	ADC_Init(ADC1, &ADC_InitStructure);

	//����ADC1ת���ĸ�ͨ������ǰ��ת��ͨ��5��˳���Ϊ1������ж��ͨ������ɨ�裬�����ֵ����ɨ���˳�򣩣����������ĵ��ʱ��Ϊ3*(1/42MHz)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_3Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_3Cycles);

	//ʹ��ADC1����
	ADC_Cmd(ADC1,ENABLE);
}

// ���������ʼ��
void photoresistor_init(void)
{
	//�򿪶˿�F��Ӳ��ʱ�ӣ����ǶԶ˿�F����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	//��ADC3��Ӳ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	
	//����PF7����ΪANģʽ������ģ���ź�ģʽ
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOF,&GPIO_InitStructure);	
	
	//����ADC1��صĲ���
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ADC����
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//ADC123��Ӳ��ʱ��Ƶ��=84MHz/2=42MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//Ĭ�ϲ�ʹ��dma
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//����������ļ��=5*(1/42MHz)
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//ʹ��ADC3����
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//ʹ����߾��Ƚ��в���
	
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//ֻɨ��һ��ͨ����ENABLE����ɨ����ͨ����
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//���ϵĶ�ͨ����ģ���źŽ���ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//����Ҫ����������ADC����
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���洢
	ADC_InitStructure.ADC_NbrOfConversion = 1;//ֻ��1��ͨ����Ҫ����ת��
	ADC_Init(ADC3, &ADC_InitStructure);

	//����ADC3ת���ĸ�ͨ������ǰ��ת��ͨ��7��˳���Ϊ1������ж��ͨ������ɨ�裬�����ֵ����ɨ���˳�򣩣����������ĵ��ʱ��Ϊ3*(1/42MHz)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_3Cycles);

	//ʹ��ADC3����
	ADC_Cmd(ADC3,ENABLE);
}

