#include "global.h"

//IChaus SPI ����
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
//	//ʹ�ܶ˿�ʱ��
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOC, ENABLE);	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);	
//	//GPIO����  SPI 5--CLK  6--MISO  7--MOSI
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7;	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;																//GPIO_Mode_AF_PP;		//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;															//50Mʱ���ٶ�
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	//GPIO����  SPI 4--NSS  CS	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;																//���⸴�����
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;															//50Mʱ���ٶ�
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
//	SPI_Cmd(SPI1, DISABLE); 																											//�����Ƚ�ֹ�����ܸı�mode
//	SPI_InitStructure.SPI_Direction =SPI_Direction_2Lines_FullDuplex; 						//����ȫ˫��
//	SPI_InitStructure.SPI_Mode =SPI_Mode_Master; 																	//�� ģʽ
//	SPI_InitStructure.SPI_DataSize =SPI_DataSize_16b; 														//8bit����
//	SPI_InitStructure.SPI_CPOL =SPI_CPOL_Low;																			//SPI_CPOL_High; //CPOL = 1����ʱ ʱ���ź�ΪH��ƽ
//	SPI_InitStructure.SPI_CPHA =SPI_CPHA_1Edge;																		//SPI_CPHA_2Edge; //CPHA = 1 ���ݲ����ӵ�X��ʱ�ӱ��ؿ�ʼ 0--1edge 1--2edge
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																			//SPI_NSS_Hard;//SPI_NSS_Soft; //CS����Ϊ���ģʽ 
//	SPI_InitStructure.SPI_BaudRatePrescaler =SPI_BaudRatePrescaler_16; 						//4��Ƶ  18MHz
//	SPI_InitStructure.SPI_FirstBit =SPI_FirstBit_MSB; 														//��bit�ȴ���
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
//  // �Ƚ����ͨ������
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				        // PWMģʽ1
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	  // ����ͨ����Ч
//  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // ����ͨ����Ч
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		    // �������
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;       // �����˵ļ���
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
//void GetPos_FromIChaus(void) //����ICHaus��դ��оƬ�����������г�
//{



//}











