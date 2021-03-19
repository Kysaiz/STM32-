/*******************************************

��ȡDHT11���¶�ֵ��ͨ��DAC�����Ӧ�ĵ�ѹֵ������ADC��ȡ�õ�ѹֵ��ת��Ϊ�¶�ֵ��

********************************************/


#include "sys.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"

#include "usart1.h"

#include "dac.h"
#include "adc.h"
#include "dht11.h"
#include "tim.h"

extern volatile uint8_t g_usart1_event;
extern volatile uint8_t g_usart1_buf[128];
extern volatile uint32_t g_usart1_cnt;

extern volatile uint32_t dht11_time;	// dht11��ȡʱ��
extern volatile uint32_t adc_delay_event;	// adc��ȡ��־λ

int main(void )
{
	uint32_t i;						// ѭ���±�
	uint32_t dac_out;				// ���¶�ת��Ϊ��ѹֵ���
	uint32_t adc_val=0;
	uint32_t adc_vol=0;
	
	int32_t dht11_ret; 				// ���ڽ�����ʪ�ȶ�ȡ��������ֵ
	uint8_t dht11_buf[5] = {0};		// ���ڴ洢��ȡ������ʪ��
	uint32_t adc_delay_time = 0;	// ADC��ʱ����ֵ
	
	dac_init();
	adc_init();
	dht11_init();
	tim3_init();
	
	//����ADC1��ʼת��
	ADC_SoftwareStartConv(ADC1);
	
	//��ʼ������1�Ĳ�����Ϊ115200bps
	//ע�⣺������յ����������룬Ҫ���PLL��
	usart1_init(115200);
	
	delay_ms(1000);
	
	printf("this is DAC 02 HOMEWORK\r\n");
	
	while(1)
	{	
		if (dht11_time >= 600)
		{
			// ��ȡ��ʪ��
			dht11_ret = dht11_read(dht11_buf);
			if (dht11_ret != 0)
			{
				printf("dht11 error code = %d\r\n", dht11_ret);
			}
			else
			{
				printf("temp from dht11 = %d.%d\r\n", dht11_buf[2], dht11_buf[3]);
			}
			
			dht11_time = 0;
			
			//����DAC�����ѹֵ
			dac_out = (dht11_buf[2] * 60 + (dht11_buf[3] * 6));
			printf("dac_out = %d\r\n", dac_out);
			
			DAC_SetChannel1Data(DAC_Align_12b_R, dac_out*4095/3300);
		}
		
		// ��ȡһ��adc
		if (adc_delay_event)
		{
			adc_val=ADC_GetConversionValue(ADC1);
			adc_vol += adc_val;
			
			adc_delay_event = 0;
			adc_delay_time++;
			
			// ÿ1�������һ��adc_vol��ת���������¶�
			if ((adc_delay_time%100) == 0)
			{
				adc_vol /= 100;
				adc_vol = (adc_vol *3300/4095);
				
				printf("adc_vol = %d\r\n", adc_vol);
				printf("temp from ADC = %d.%d\r\n\n", adc_vol/60, (adc_vol%60)/6);
				
				adc_delay_time = 0;
				adc_vol = 0;
			}
			
		}
		
	}
	
	
}
