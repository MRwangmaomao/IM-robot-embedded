#ifndef _BUMPRE_H_
#define _BUMPRE_H_

#include "config.h"

#define BUMPER_FRONT GPIO_ReadInputDataBit(RIKI_BUMPER_GPIO_PORT,RIKI_BUMPER_FRONT_PIN)//��ȡ����0
#define BUMPER_BACK GPIO_ReadInputDataBit(RIKI_BUMPER_GPIO_PORT,RIKI_BUMPER_BACK_PIN)//��ȡ����1

void bumper_init();

#endif //_BUMPER_H_
