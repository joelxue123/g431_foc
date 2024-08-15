#include "led.h"
#include "stm32g4xx_hal.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;


	//使能端口时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	//LED1_OFF();
	//LED2_OFF();  
	//GPIO设置
	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

	#if 1 

	GPIO_InitStructure.Pin  = GPIO_PIN_4;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	#endif 
	
	GPIO_InitStructure.Pin  = GPIO_PIN_12|GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin  = GPIO_PIN_11;// MP6540  BBL_nfault pin
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin  = GPIO_PIN_12;// MP6540  BBL_nfault pin
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
	
	
	GPIO_InitStructure.Pin  = GPIO_PIN_14;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//void toggle_led(void)
//{
//	uint8_t val;
//	val = GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_2);
//	val = 1- val;
//	GPIO_WriteBit(GPIOB,GPIO_Pin_2,val);
//}
