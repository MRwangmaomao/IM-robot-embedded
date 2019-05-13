

#include "bumper.h"

void bumper_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	
 	RCC_APB2PeriphClockCmd(RIKI_BUMPER_GPIO_CLK,ENABLE);//Ê¹ÄÜPORTÊ±ÖÓ

	GPIO_InitStructure.GPIO_Pin  = RIKI_BUMPER_FRONT_PIN|RIKI_BUMPER_BACK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
 	GPIO_Init(RIKI_BUMPER_GPIO_PORT, &GPIO_InitStructure);  
	 
}
