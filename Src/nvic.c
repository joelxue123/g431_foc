#include "stm32g4xx_hal.h"
#include "nvic.h"
#include "global.h"
void NVIC_Configuration(void)
{


  /* Set the Vector Table base location at 0x08000000 */

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
	

	

		
//				HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 1);
//    HAL_NVIC_EnableIRQ(USART1_IRQn);
//		
//		HAL_NVIC_SetPriority(TIM3_IRQn, 1, 1);
//    HAL_NVIC_EnableIRQ(USART1_IRQn);
		
}
