#ifndef _BEEP_H_
#define _BEEP_H_

#include "config.h"
 

void bumper_init(u16 arr,u16 psc);
void beep_tune(u16 arr,u16 psc);
void beep_bringup();


#endif //_BEEP_H_
