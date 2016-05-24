#include "TMER_CFG.h"
uint16_t Duty_Temp;
uint16_t T2ms_Temp,T100ms_Temp;
uint8_t T2ms_Flag=0,T100ms_Flag=0;

void Delay(uint32_t nCount)
{
  for(;nCount!=0;nCount--);
}

void TMER_Iinitialization(void)
{
	//GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;
	//Interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
		//Tmer
	TIM_BDTRInitTypeDef  TIM_BDTRInitStructure;				   //死区刹车结构体变量定义
	TIM_OCInitTypeDef TIM_OCInitStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;                      //定义结构体变量

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
/**************************************************************************
//---------------------------------TMER1 PWM-------------------------------
***************************************************************************/
	PWM_Period=((SystemCoreClock / (2*PWM_FREQ) ) - 1);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  /* TIM2 Configuration */
  TIM_DeInit(TIM1);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	/* GPIOA Configuration: Channel 1, 2, 3, 4 and Channel 1N as alternate function push-pull */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIOB Configuration: Channel 2N and 3N as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_2);					//TRIP
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_2); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_2);

	/*----------------------------------------------------------------------- */
	/* Compute the value to be set in ARR regiter to generate signal frequency at 12 Khz */
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;					   //TIM基本初始化
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;//中央对齐计数模式
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;

	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 		   //TIM输出通道初始化
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                  
	TIM_OCInitStructure.TIM_Pulse =200; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;          

	TIM_OC1Init(TIM1,&TIM_OCInitStructure); 
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);

  TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 		   //TIM输出通道4初始化，用来触发AD采样
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                   
	TIM_OCInitStructure.TIM_Pulse =PWM_PERIOD-24; //100; 		//
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	TIM_OC4Init(TIM1,&TIM_OCInitStructure); 
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;	//死区刹车初始化
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; 
	TIM_BDTRInitStructure.TIM_DeadTime = DEADTIME;							//96-2us
	TIM_BDTRInitStructure.TIM_Break =TIM_Break_Enable;	 // TIM_Break_Disable;//
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);

	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);   //使能捕获比较寄存器预装载（通道1）

	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);	 //使能捕获比较寄存器预装载（通道2）

	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);	 //使能捕获比较寄存器预装载（通道3）

  /* Enable the TIM1 Trigger and commutation interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
	
  TIM_ClearFlag(TIM1, TIM_FLAG_Update); 							// 清除TIM2的更新标志位 ；
	TIM_ClearFlag(TIM1, TIM_FLAG_Break);
	TIM_ClearFlag(TIM1, TIM_FLAG_COM);
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);	
}

