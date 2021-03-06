#ifndef __USART_H__
#define __USART_H__

#include "config.h"
#include "RingBuffer.h"

class HardwareSerial {
public:
	int distance_sonar_i;
	uint8_t distance_buffer[7];
	bool sonar_update_flag;
	HardwareSerial(Serial_TypeDef _Serial=SERIAL1);
	~HardwareSerial(){};
	void begin(uint32_t baud);

	uint32_t available(void);
	uint8_t read(void);
	void flush(void);
	uint32_t write(uint8_t ch);
	void print(const char *format, ...);
	void putstr(const char *str);
	void irq();
	void sonar_irq();
		
protected: 
	RingBuffer rx_buffer; 
	Serial_TypeDef Serial;
};

#endif //__USART_H__
