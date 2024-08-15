#include "global.h"

//IChaus SPI 配置
//void SPI_IChaus_Configuration(void)
//{
//	GPIO_InitTypeDef 	GPIO_InitStructure;
//	SPI_InitTypeDef 	SPI_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//  TIM_OCInitTypeDef  TIM_OCInitStructure;
//	DMA_InitTypeDef DMA_InitStructure;	
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//  TIM_OCStructInit(&TIM_OCInitStructure);	
//  GPIO_StructInit(&GPIO_InitStructure);
//  SPI_StructInit(&SPI_InitStructure);	
//	//使能端口时钟
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOC, ENABLE);	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);	
//	//GPIO设置  SPI 5--CLK  6--MISO  7--MOSI
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7;	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;																//GPIO_Mode_AF_PP;		//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;															//50M时钟速度
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	//GPIO设置  SPI 4--NSS  CS	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;																//推免复用输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;															//50M时钟速度
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
//	SPI_Cmd(SPI1, DISABLE); 																											//必须先禁止，才能改变mode
//	SPI_InitStructure.SPI_Direction =SPI_Direction_2Lines_FullDuplex; 						//两线全双工
//	SPI_InitStructure.SPI_Mode =SPI_Mode_Master; 																	//主 模式
//	SPI_InitStructure.SPI_DataSize =SPI_DataSize_16b; 														//8bit数据
//	SPI_InitStructure.SPI_CPOL =SPI_CPOL_Low;																			//SPI_CPOL_High; //CPOL = 1空闲时 时钟信号为H电平
//	SPI_InitStructure.SPI_CPHA =SPI_CPHA_1Edge;																		//SPI_CPHA_2Edge; //CPHA = 1 数据采样从第X个时钟边沿开始 0--1edge 1--2edge
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																			//SPI_NSS_Hard;//SPI_NSS_Soft; //CS引脚为软件模式 
//	SPI_InitStructure.SPI_BaudRatePrescaler =SPI_BaudRatePrescaler_16; 						//4分频  18MHz
//	SPI_InitStructure.SPI_FirstBit =SPI_FirstBit_MSB; 														//高bit先传输
//	SPI_InitStructure.SPI_CRCPolynomial =7; 																			//CRC7   
//	SPI_Init(SPI1,&SPI_InitStructure);   
//	SPI_Cmd(SPI1, ENABLE);	
//	//SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);	
//	SPI_SSOutputCmd(SPI2, DISABLE);		
//	TIM_DeInit(TIM3);
//  TIM_TimeBaseStructure.TIM_Prescaler = 0;						               
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;		
//  TIM_TimeBaseStructure.TIM_Period = TIM3_PERIOD;					                    
//  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
//  // 比较输出通道设置
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				        // PWM模式1
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	  // 正向通道有效
//  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // 反向通道有效
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		    // 输出极性
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;       // 互补端的极性
//  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
//  TIM_OCInitStructure.TIM_Pulse = TIM3_CCR1;
//  TIM_OC1Init(TIM3,&TIM_OCInitStructure); 
//  TIM_OCInitStructure.TIM_Pulse = TIM3_CCR3; 
//  TIM_OC3Init(TIM3,&TIM_OCInitStructure);	
//	TIM_DMACmd(TIM3,TIM_DMA_CC3,ENABLE);
//	DMA_DeInit(DMA1_Channel2);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Zero_s16;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//  DMA_InitStructure.DMA_BufferSize = 1;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
//  DMA_Cmd(DMA1_Channel2, ENABLE);	
//}
//void GetPos_FromIChaus(void) //利用ICHaus磁栅和芯片计算驱动器行程
//{



//}











