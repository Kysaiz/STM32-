#ifndef __DHT11__
#define __DHT11__

extern void dht11_init(void);



extern int32_t dht11_read_data(uint8_t *pbuf);

#endif
