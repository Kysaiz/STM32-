#include "dht11.h"

static GPIO_InitTypeDef		GPIO_InitStructure;

void dht11_init()
{
	//ʹ�ܶ˿�GӲ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//��ʼ��
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	
	//PG9����ߵ�ƽ���������ŵĳ�ʼ��ƽ״̬
	PGout(9)=1;	
}

int32_t dht11_read(uint8_t *pbuf)
{
	uint32_t t = 0;
	int32_t i, j;
	uint8_t d = 0;
	uint8_t *p = pbuf;
	uint8_t check_sum = 0;
	
	// ����PG9��ʼģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // ����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // ��©���ģʽ
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStructure);	
	
	PGout(9)=0;
	delay_ms(18);
	PGout(9)=1;
	delay_us(30);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // ����Ϊ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	// PG9�ȴ��͵�ƽ���֣���ӳ�ʱ����
	while (PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 4000) // ��t����4msʱ��ʾ��ʱ���˳�����������-1
		{
			return -1;
		}
	}
	
	// ���͵�ƽ�ĳ���ʱ��
	t = 0;
	while (!PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 100) // ����ʱ��ͼ��tӦ��Ϊ80us
		{
			return -2;
		}
	}
	
	// ���ߵ�ƽ�ĳ���ʱ��
	t = 0;
	while (PGin(9))
	{
		t++;
		delay_us(1);
		
		if (t >= 100) // ����ʱ��ͼ��tӦ��Ϊ80us
		{
			return -3;
		}
	}
	
	// ��ʼ��������
	for (i = 0; i < 5; i++)
	{
		// ����1���ֽڣ���λ���Ƚ�������
		d = 0;
		for (j = 7; j >= 0; j--)
		{
			// PG9�ȴ��ߵ�ƽ���֣���ӳ�ʱ����
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
			
			// ��ʱ40~60us
			delay_us(40);
			
			// �ж����ŵĵ�ƽ
			if (PGin(9))
			{
				//��d������Ӧ��bitλ��1
				//0x01 1<<0:0000 0001
				//0x02 1<<1:0000 0010
				//.......
				//0x80 1<<7:1000 0000
				d|=1<<j;
				
				//�ȴ�ʣ�µĸߵ�ƽ�������
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
	
	// У������
	check_sum = (p[0] + p[1] + p[2] + p[3])&0xFF;
	
	// ���У�鲻���������-6
	if (check_sum != p[4])
		return -6;
			
	return 0;
	
}
