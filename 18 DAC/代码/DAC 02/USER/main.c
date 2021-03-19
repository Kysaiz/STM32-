/*******************************************

读取DHT11的温度值，通过DAC输出对应的电压值，接着ADC读取该电压值，转换为温度值。

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

extern volatile uint32_t dht11_time;	// dht11读取时间
extern volatile uint32_t adc_delay_event;	// adc读取标志位

int main(void )
{
	uint32_t i;						// 循环下标
	uint32_t dac_out;				// 将温度转换为电压值输出
	uint32_t adc_val=0;
	uint32_t adc_vol=0;
	
	int32_t dht11_ret; 				// 用于接收温湿度读取函数返回值
	uint8_t dht11_buf[5] = {0};		// 用于存储读取到的温湿度
	uint32_t adc_delay_time = 0;	// ADC延时计数值
	
	dac_init();
	adc_init();
	dht11_init();
	tim3_init();
	
	//启动ADC1开始转换
	ADC_SoftwareStartConv(ADC1);
	
	//初始化串口1的波特率为115200bps
	//注意：如果接收的数据是乱码，要检查PLL。
	usart1_init(115200);
	
	delay_ms(1000);
	
	printf("this is DAC 02 HOMEWORK\r\n");
	
	while(1)
	{	
		if (dht11_time >= 600)
		{
			// 读取温湿度
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
			
			//设置DAC输出电压值
			dac_out = (dht11_buf[2] * 60 + (dht11_buf[3] * 6));
			printf("dac_out = %d\r\n", dac_out);
			
			DAC_SetChannel1Data(DAC_Align_12b_R, dac_out*4095/3300);
		}
		
		// 读取一次adc
		if (adc_delay_event)
		{
			adc_val=ADC_GetConversionValue(ADC1);
			adc_vol += adc_val;
			
			adc_delay_event = 0;
			adc_delay_time++;
			
			// 每1秒钟输出一次adc_vol和转换出来的温度
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
