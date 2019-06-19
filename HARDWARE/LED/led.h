#ifndef __LED_H
#define __LED_H	 
#include "header.h"


#define LED0 PBout(3)// PB5
#define LED0_Status GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3)

void LED_Init(void);//≥ı ºªØ

		 				    
#endif
