#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H
#include <stdint.h>
#include "stm32f10x_conf.h"

void Ultrasonic_Init(void);
uint8_t UlsResultVerification(DWORD result);



#endif

