#ifndef _LED_H_
#define _LED_H_
 
#include "config.h"


 
class Led {
public:
	void init();
	void red_on_off(bool status);
	void green_on_off(bool status);
	void green_bar_on_off(unsigned char index);
};

#endif //_LED_H_
