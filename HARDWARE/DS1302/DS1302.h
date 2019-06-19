#ifndef __DS1302_H
#define __DS1302_H
#include "sys.h"

void DS1302_PHY_Init(void);
void Init_DS1302(void);
void Readburst(unsigned char data[]);
unsigned char Readbyte(u8 addr);
void DS1302_DateSetup(u8 *date);
#endif

