#ifndef __LED_H__
#define __LED_H__

#include "stm32g4xx_hal.h"
#include "global.h"
/*
  LED1  : PC10 (低电平点亮，高电平熄灭)
  LED2  : PC11 (低电平点亮，高电平熄灭)
*/
#define RCC_GPIO_LED1 	  RCC_AHB1Periph_GPIOC
#define RCC_GPIO_LED2 	  RCC_AHB1Periph_GPIOC

#define GPIO_PORT_LED1  GPIOC
#define GPIO_PIN_LED1	  GPIO_Pin_7

#define GPIO_PORT_LED2  GPIOC
#define GPIO_PIN_LED2	  GPIO_Pin_8



#define ICMU_POWER_OFF (GPIOC->ODR &= ~GPIO_Pin_14)
#define ICMU_POWER_ON (GPIOC->ODR |= GPIO_Pin_14)

void toggle_led(void);

/* 供外部调用的函数声明 */
extern void LED_Init(void);

#endif

