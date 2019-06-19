#ifndef __HEADER_H
#define __HEADER_H

#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x.h"
#include "usart.h"
#include "can.h"
#include "canard.h"
#include "delay.h"
#include "ds18b20.h"
#include "adc.h"
#include "I2C.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "core_cm3.h"
#include "ultrasonic.h"
#include "DS1302.h"
#include "MPU9250.h"
#include "FreeRTOS.h"
#include "event_groups.h"

#define DEBUG 	1
#define		NumberOfSamplingChannel		3
/*
define the controlling ports for ultrasonic and infrared sensors
*/
#define WATCHDOG  GPIO_Pin_0
#define WHDG_IO		PCout(0)

#define TRG6 	GPIO_Pin_8	//PC8
#define TRG5	GPIO_Pin_14	//PB14
#define TRG4 	GPIO_Pin_13	//PB13
#define TRG3 	GPIO_Pin_15		//PB15
#define TRG2 	GPIO_Pin_7		//PC7
#define TRG1 	GPIO_Pin_9   	//PC9

#define ECHO6	GPIO_Pin_8	//PA8
#define ECHO5	GPIO_Pin_6	//PC6
#define ECHO4 	GPIO_Pin_2	//PA2
#define ECHO3	GPIO_Pin_0	//PA0
#define ECHO2	GPIO_Pin_6	//PA6
#define ECHO1 	GPIO_Pin_6	//PB6


/*
	1. 依据板子上丝印顺序定义控制顺序
*/
#define TRG_6		PCout(9)
#define TRG_5		PCout(8)	
#define	TRG_4 		PBout(14)
#define	TRG_3 		PCout(7)
#define	TRG_2 		PBout(15)
#define	TRG_1 		PBout(13)

#define ECHO1_EVENTBIT (1<<0)
#define ECHO2_EVENTBIT (1<<1)
#define ECHO3_EVENTBIT (1<<2)
#define ECHO4_EVENTBIT (1<<3)
#define ECHO5_EVENTBIT (1<<4)
#define ECHO6_EVENTBIT (1<<5)
#define INFRA_EVENTBIT (1<<6)
#define MPU_EVENTBIT   (1<<7)
#define EVENTBIT_ALL (ECHO1_EVENTBIT|ECHO2_EVENTBIT|ECHO3_EVENTBIT|ECHO4_EVENTBIT|ECHO5_EVENTBIT|ECHO6_EVENTBIT|INFRA_EVENTBIT|MPU_EVENTBIT)


#define ULTRS_DISTANCE_MIN 20


/********************************************************************************************************************************/
typedef struct 
{
	char      model_name[26];//
	uint8_t 	firmware_version_H8bit;//
	uint8_t 	firnware_version_L8bit;//
	uint8_t 	hardware_version_H8bit;//
	uint8_t 	hardware_version_L8bit;//
	uint64_t 	serial_number;//
}__attribute__((packed)) TDeviceInfo;

typedef struct{
	WORD m_PulseWidth;
	WORD m_PulseWidth_Prev;
	BYTE m_Flag;
	DWORD m_Distance;   // unit: mm
}Sonar_Vect;

typedef struct
{
	WORD m_Pulse;
	BYTE m_Flag;
	u16 m_DataVector[10];
	u16 m_buf[10];
	u32 m_Aveage;
	u32 m_Sum;
	u8 m_Count;
	u16 m_Distance; //unit : mm
}Infra_Vect;

typedef struct
{
	Infra_Vect Infra01;
	Infra_Vect Infra02;
	u8 m_Infra_Flag;
}Infra_TypeDef;


typedef struct
{
	DWORD m_SonarTime_Interval;
	DWORD m_InfarTime_Interval;
	DWORD m_TempTime_Interval;
	DWORD m_WatchDog_Interval;
	DWORD m_Broadcast_Interval;
}GlobTimer_TypeDef;

typedef struct
{
	u8 imu_Reg_context[4];
	short gyro[3];
	short accel_short[3];
	short sensors;
	unsigned char more;
	long accel[3];
	long quat[4];
	short mag[3];
	long temperature;
	unsigned long sensor_timestamp;
	u8 ID;
	uint32_t error_cnt;
	u8 startup_flag;
	uint16_t imu_pwr_rst_cnt;

}GlobMPU9250;


typedef struct
{
	u16 m_Temperaturer_Prev;
	u16 m_ErrCount;
	u16 m_Temperaturer;
	u8  m_Time[7];
}GlobTemp_TypeDef;

typedef struct
{
	u16 m_ID;
}CANID_TypeDef;



typedef struct
{
	Sonar_Vect Sonar01;
	Sonar_Vect Sonar02;
	Sonar_Vect Sonar03;
	Sonar_Vect Sonar04;
	Sonar_Vect Sonar05;
	Sonar_Vect Sonar06;
	Infra_TypeDef InfraredSensor;
	GlobTemp_TypeDef Temperaturer;
	GlobTimer_TypeDef Sensor_timer;
	volatile u16 RawData[NumberOfSamplingChannel];
	CANID_TypeDef CanID;
	GlobMPU9250 imuData;
	u8 					SonarEnable;
	u8					IREnable;
	u8					IMUEnable;
	u8					RESETEnable;
}GlobalSensor;


//struct platform_data_s {
//    signed char orientation[9];
//};

//struct platform_data_s gyro_pdata = {
//    .orientation = { 1, 0, 0,
//                     0, 1, 0,
//                     0, 0, 1}
//};
//extern struct platform_data_s gyro_pdata;
extern struct int_param_s int_param;
extern GlobalSensor gSensor;
extern volatile uint32_t g_ul_ms_ticks;
extern EventGroupHandle_t EventGroupHandle;
#endif
