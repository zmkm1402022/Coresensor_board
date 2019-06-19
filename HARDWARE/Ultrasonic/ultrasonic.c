#include "ultrasonic.h"
#include "header.h"
#include "FreeRTOS.h"
#include "event_groups.h"




void Sonar06_init(void);
void Sonar05_init(void);
void Sonar04_init(void);
void Sonar03_init(void);
void Sonar02_init(void);
void Sonar01_init(void);


void Ultrasonic_Init(void)
{
	Sonar06_init();
	Sonar05_init();
	Sonar04_init();
	Sonar03_init();
	Sonar02_init();
	Sonar01_init();
}


void Sonar06_init(void)
{
	TIM_ICInitTypeDef TIM_ICInitStructure; 
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	/* TIM3 clock enable */ 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	/* GPIOA clock enable */ 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = ECHO6;							   //GPIO??
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF;	  //
		TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/500000-1; // ?±??????=10K
		TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
	
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
	//	TIM_ICInit(TIM2, &TIM_ICInitStructure);  
		TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);

		TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
	
		/* Select the slave Mode: Reset Mode */ 
		TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
	
		/* Enable the Master/Slave Mode */ 
		TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	
		/* TIM enable counter */ 
		TIM_Cmd(TIM1, ENABLE);
	
		/* Enable the CC2 Interrupt Request */ 
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; 					
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE); 

}


void Sonar05_init(void)
{
	TIM_ICInitTypeDef TIM_ICInitStructure; 
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	/* TIM3 clock enable */ 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	
	/* GPIOA clock enable */ 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = ECHO5;							   //GPIO??
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF;	  //
		TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/500000-1; // ?±??????=10K
		TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
	
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
	//	TIM_ICInit(TIM2, &TIM_ICInitStructure);  
		TIM_PWMIConfig(TIM8, &TIM_ICInitStructure);

		TIM_SelectInputTrigger(TIM8, TIM_TS_TI1FP1);
	
		/* Select the slave Mode: Reset Mode */ 
		TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);
	
		/* Enable the Master/Slave Mode */ 
		TIM_SelectMasterSlaveMode(TIM8, TIM_MasterSlaveMode_Enable);
	
		/* TIM enable counter */ 
		TIM_Cmd(TIM8, ENABLE);
	
		/* Enable the CC2 Interrupt Request */ 
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn; 					
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE); 


}

/*
	IRQ priority should be set high as possible.
*/
void Sonar04_init(void) 
{ 
	TIM_ICInitTypeDef TIM_ICInitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
/* TIM2 clock enable */ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

/* GPIOA clock enable */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = ECHO4;                               //GPIO??
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;    //
	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/500000-1;  // ?±??????=10K
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 


	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	
	TIM_ICInit(TIM2, &TIM_ICInitStructure);  
	TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);
	/* Enable the CC2 Interrupt Request */ 
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                     
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE); 
	TIM2->CR1|=0x01;   
}


void Sonar03_init(void) 
{ 
	TIM_ICInitTypeDef TIM_ICInitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
/* TIM3 clock enable */ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

/* GPIOA clock enable */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = ECHO3;                               //GPIO??
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;    //
	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/500000-1; // ?±??????=10K
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
//	TIM_ICInit(TIM2, &TIM_ICInitStructure);  
	TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);


	TIM_SelectInputTrigger(TIM5, TIM_TS_TI1FP1);

	/* Select the slave Mode: Reset Mode */ 
	TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */ 
	TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);

	/* TIM enable counter */ 
	TIM_Cmd(TIM5, ENABLE);

	/* Enable the CC2 Interrupt Request */ 
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;                     
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE); 
}

void Sonar02_init(void) 
{ 
	TIM_ICInitTypeDef TIM_ICInitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
/* TIM3 clock enable */ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

/* GPIOA clock enable */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = ECHO2;                               //GPIO??
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;    //
	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/500000-1; // ?±??????=500K
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);


	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);

	/* Select the slave Mode: Reset Mode */ 
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */ 
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

	/* TIM enable counter */ 
	TIM_Cmd(TIM3, ENABLE);

	/* Enable the CC2 Interrupt Request */ 
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                     
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE); 
}


void Sonar01_init(void) 
{ 
	TIM_ICInitTypeDef TIM_ICInitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
/* TIM3 clock enable */ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

/* GPIOA clock enable */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = ECHO1;                               //GPIO??
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;    //
	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/500000-1;  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);


	TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);

	/* Select the slave Mode: Reset Mode */ 
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */ 
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);

	/* TIM enable counter */ 
	TIM_Cmd(TIM4, ENABLE);

	/* Enable the CC2 Interrupt Request */ 
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                     
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE); 
}

void TIM1_CC_IRQHandler(void)
{
//	BaseType_t Result, xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
	if(TIM_GetFlagStatus(TIM1, TIM_FLAG_CC2) == SET)
	{
		gSensor.Sonar05.m_PulseWidth = TIM_GetCapture2(TIM1);
		gSensor.Sonar05.m_Distance = gSensor.Sonar05.m_PulseWidth*344/1000;
//		Result = xEventGroupSetBitsFromISR(EventGroupHandle,ECHO6_EVENTBIT,&xHigherPriorityTaskWoken);
//		if(Result != pdFAIL)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		TIM_ClearITPendingBit(TIM1, TIM_FLAG_CC2);
		TIM_ClearFlag(TIM1, TIM_FLAG_CC2);
	}
}

void TIM8_CC_IRQHandler(void)
{
//	BaseType_t Result, xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
	if(TIM_GetFlagStatus(TIM8, TIM_FLAG_CC2) == SET)
	{
		gSensor.Sonar04.m_PulseWidth = TIM_GetCapture2(TIM8);
		gSensor.Sonar04.m_Distance = gSensor.Sonar04.m_PulseWidth*344/1000;
//		Result = xEventGroupSetBitsFromISR(EventGroupHandle,ECHO5_EVENTBIT,&xHigherPriorityTaskWoken);
//		if(Result != pdFAIL)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		TIM_ClearITPendingBit(TIM8, TIM_FLAG_CC2);
		TIM_ClearFlag(TIM8, TIM_FLAG_CC2);
	}
}


void TIM4_IRQHandler(void) 
{
//	BaseType_t Result, xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;	
	if(TIM_GetFlagStatus(TIM4,TIM_FLAG_CC2) == SET) 
		{ 
		gSensor.Sonar06.m_PulseWidth = TIM_GetCapture2(TIM4); 
		gSensor.Sonar06.m_Distance = gSensor.Sonar06.m_PulseWidth*344/1000;   // unit: mm
//		Result = xEventGroupSetBitsFromISR(EventGroupHandle,ECHO1_EVENTBIT,&xHigherPriorityTaskWoken);
//		if(Result != pdFAIL)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		TIM_ClearFlag(TIM4,TIM_FLAG_CC2);
		}
}

void TIM5_IRQHandler(void) 
{ 
//	BaseType_t Result, xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
	if(TIM_GetFlagStatus(TIM5,TIM_FLAG_CC2) == SET) 
		{ 
		gSensor.Sonar02.m_PulseWidth = TIM_GetCapture2(TIM5); 
		gSensor.Sonar02.m_Distance = gSensor.Sonar02.m_PulseWidth*344/1000;   // unit: mm
//		Result = xEventGroupSetBitsFromISR(EventGroupHandle,ECHO3_EVENTBIT,&xHigherPriorityTaskWoken);
//		if(Result != pdFAIL)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			TIM_ClearFlag(TIM5,TIM_FLAG_CC2);
		}
}

void TIM3_IRQHandler(void) 
{ 
//	BaseType_t Result, xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
	if(TIM_GetFlagStatus(TIM3,TIM_FLAG_CC2) == SET) 
		{ 
		gSensor.Sonar03.m_PulseWidth = TIM_GetCapture2(TIM3); 
		gSensor.Sonar03.m_Distance = gSensor.Sonar03.m_PulseWidth*344/1000;   // unit: mm
//		Result = xEventGroupSetBitsFromISR(EventGroupHandle,ECHO2_EVENTBIT,&xHigherPriorityTaskWoken);
//		if(Result != pdFAIL)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			TIM_ClearFlag(TIM3,TIM_FLAG_CC2);
		}
}

void TIM2_IRQHandler(void) 
{ 
//	BaseType_t Result, xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
	uint16_t capturetime;
	if(TIM_GetFlagStatus(TIM2,TIM_FLAG_CC3) == SET) 
		{ 
			capturetime = TIM_GetCapture3(TIM2);
			if (gSensor.Sonar01.m_Flag == RESET){
				gSensor.Sonar01.m_PulseWidth_Prev = capturetime; 
				TIM2->CCER|=1<<9;   //?è??????????????
				gSensor.Sonar01.m_Flag = SET;
			}
			else
			{
				if (capturetime >gSensor.Sonar01.m_PulseWidth_Prev)
				gSensor.Sonar01.m_PulseWidth = capturetime-gSensor.Sonar01.m_PulseWidth_Prev;
				else
					gSensor.Sonar01.m_PulseWidth = 0xFFFF+capturetime-gSensor.Sonar01.m_PulseWidth_Prev;
				gSensor.Sonar01.m_Distance = gSensor.Sonar01.m_PulseWidth*344/1000;    // unit: mm
				
				gSensor.Sonar01.m_Flag = RESET;
				gSensor.Sonar01.m_PulseWidth_Prev = 0;		
				TIM2->CCER &= ~(1<<9);
				TIM_SetCounter(TIM2, 0);				
			}
			
			TIM_ClearFlag(TIM2,TIM_FLAG_CC3);
		}
}

/*
[INPUT] result 的单位是 mm
[return] : unit cm
*/
uint8_t UlsResultVerification(DWORD result)
{
	if (result > 2000)
		return 0xFF;
	else if (result <= ULTRS_DISTANCE_MIN)
		return ULTRS_DISTANCE_MIN;
	else if (result == 0)
		return 0xFE;
	else
		return ((uint8_t)result/10);
}


