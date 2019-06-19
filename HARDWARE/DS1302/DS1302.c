#include "DS1302.h"
#include "delay.h"

#define DS1302_SCLK    PAout(7)
#define DS1302_CE   PCout(5)

#define DS1302_IOOUT    PCout(4)
#define DS1302_IOIN    PCin(4)

#define DS1302_IO_IN()  {GPIOC->CRL&=0XFFF0FFFF;GPIOC->CRL|=8<<16;}
#define DS1302_IO_OUT() {GPIOC->CRL&=0XFFF0FFFF;GPIOC->CRL|=3<<16;}


void DS1302_PHY_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE); 
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;				
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		  
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_4); 
	GPIO_ResetBits(GPIOC,GPIO_Pin_5); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);	

}
//1302????,  ????????
void Rst1302(void)
{
    DS1302_SCLK=0; 
    DS1302_CE=0; 
		delay_us(5);
    DS1302_CE=1; 
}

void End1302(void) //????????
{
	DS1302_SCLK=0; 
	DS1302_CE=0; 
}

void IO_IN(void) //GPIOD0???????
{
	DS1302_IO_OUT();
}

void IO_OUT(void) //GPIOD0???????
{
 DS1302_IO_IN();
}

void Write1302(unsigned char addr)
{
	unsigned char i,temp;
	temp=addr;
	for (i=8;i>0;i--)	//????
	{
		DS1302_SCLK = 0;
		delay_us(2);
		if (temp & 0x01)
			DS1302_IOOUT = 1;
		else
			DS1302_IOOUT = 0;
		delay_us(2);
		DS1302_SCLK = 1;
		temp >>= 1;
		
	}
}

 

unsigned char Read1302(void)
{
	unsigned char dat1,dat2,readvalue;
	unsigned char i;
	readvalue=0x00;	
	for (i=8;i>0;i--)
	{
		DS1302_SCLK = 0;
		delay_us(10);
		readvalue>>=1;
		if (DS1302_IOIN)
			readvalue|=0x80;
		
		DS1302_SCLK = 1;
		delay_us(10);
	}		
	dat1=readvalue;
	dat2=(dat1/16);
	dat1=dat1%16;
	dat1=dat1+dat2*10;
	return(dat1); 	// 返回的日期是十进制数
}


void Init_DS1302(void)
{
	DS1302_PHY_Init();
	
//	IO_IN()	;
//	delay_us(10);
//	Rst1302();
//	Write1302(0x8e);
//	Write1302(0x00);
//	End1302();
//	delay_us(10);
//	Rst1302();
//	Write1302(0x80);
//	Write1302(0x00);
//	End1302();
//	Rst1302();
//	Write1302(0x82);
//	Write1302(0x26);
//	End1302();
//	Rst1302();
//	Write1302(0x84);
//	Write1302(0x21);
//	End1302();
//	Rst1302();
//	Write1302(0x86);
//	Write1302(0x13);
//	End1302();
//	Rst1302();
//	Write1302(0x88);
//	Write1302(0x10);
//	End1302();
//	Rst1302();
//	Write1302(0x8A);
//	Write1302(0x07);
//	End1302();
//	Rst1302();
//	Write1302(0x8C);
//	Write1302(0x18);
//	End1302();
//	Rst1302();
//	Write1302(0xbe);
//	Write1302(0x00); //second
//	Write1302(0x10); //minutes
//	Write1302(0x05); //hour
//	Write1302(0x20); //date (1-31)
//	Write1302(0x11); //month 
//	Write1302(0x05); //day (1-7: 1-sun; 2-monday;.....)
//	Write1302(0x14); //year
//	End1302();


//	Write1320(Write_Prot,0x00);

//	Write1320(Write_Year,0x14);

//	Write1320(Write_Mon,0x11);

//	Write1320(Write_Date,0x03);

//	Write1320(Write_Hr,0x12);

//	Write1320(Write_Min,0x59);

//	Write1320(Write_Sec,0x59);

//	Write1320(Write_Prot,0x80);
}

void DS1302_DateSetup(u8 *date)
{
	IO_IN()	;
	delay_us(10);
	Rst1302();
	Write1302(0x8e);
	Write1302(0x00);
	End1302();
	delay_us(10);
	Rst1302();
	Write1302(0x80);  // 秒
	Write1302(*(date+6));
	End1302();
	Rst1302();
	Write1302(0x82);
	Write1302(*(date+5));  //分钟
	End1302();
	Rst1302();
	Write1302(0x84);
	Write1302(*(date+4));  //小时
	End1302();
	Rst1302();
	Write1302(0x86);
	Write1302(*(date+3));  //日
	End1302();
	Rst1302();
	Write1302(0x88);
	Write1302(*(date+2));  //月
	End1302();
	Rst1302();
	Write1302(0x8A);
	Write1302(*(date+1));   //星期几
	End1302();
	Rst1302();
	Write1302(0x8C);
	Write1302(*date);   //年 
	End1302();
}

void Readburst(unsigned char  data[])
{
	IO_IN();
 	Rst1302();
 	Write1302(0xBF);
	IO_OUT();
	delay_us(10);
 	data[0]=Read1302();// sec
 	data[1]=Read1302();//min
 	data[2]=Read1302();//hrs
 	data[3]=Read1302();//date
 	data[4]=Read1302();//month
 	data[5]=Read1302();//day
 	data[6]=Read1302();//year
	End1302();
}

unsigned char Readbyte(u8 addr)
{
 	unsigned char temp_data;
 	IO_IN();
	delay_us(10);
	Rst1302();
	Write1302(addr);

 	IO_OUT();
	temp_data=Read1302();
	End1302();
	return (temp_data);

}

