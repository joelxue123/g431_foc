#include "stm32g4xx_hal.h"

#define IChaus_SPI_PERIOD_US	5 						//磁编SPI的采样周期微秒
#define IChaus_SPI_CSHIGH_US	1 						//CS高电平时间微秒
#define IChaus_SPI_CLK_DELAY 18 						// 18:250ns 9:125ns

#define TIM3_PERIOD	(IChaus_SPI_PERIOD_US*72) 
#define TIM3_CCR1 (IChaus_SPI_CSHIGH_US*72) 
#define TIM3_CCR3 (TIM3_CCR1 + IChaus_SPI_CLK_DELAY) 
extern void SPI_IChaus_Configuration(void);




