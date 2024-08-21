/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "global.h"
#include "limit_pos_detection.h"
#include "cmsis_os.h"
#include "sensorless.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


osThreadId defaultTaskHandle;


void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
 void MX_ADC1_Init(void);
 void MX_ADC2_Init(void);
 void MX_TIM1_Init(void);
 void MX_USART1_UART_Init(void);
 void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

static int32_t s_Count_Rec = 0;
static uint16_t buf[32];
int main(void)
{
	volatile int i = 0;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	NVIC_Configuration();
  /* USER CODE BEGIN SysInit */
	Flash_Init();    
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	LED_Init();
  init_motor(&motor_);
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
//	hall_Configuration();
  PWM_Init();
	if (!CurrentDemarcate())   
	{
		g_CmdMap[CMD_ERROR] |=  ERROR_MASK_CURRENT_CLIB_FAULT;
	}

  MX_USART1_UART_Init();
//	UnityPrint("start uart\n");
	work_var_updata();
  /* USER CODE BEGIN 2 */
	
	//MX_SPI3_Init();
	//MX_SPI1_Init();
	
  /* USER CODE END 2 */
	g_CmdMap[SYS_MU_COMM_TO_PC] = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 // osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
 // osKernelStart();

  while (1)
  {
	//	HAL_UART_Transmit_DMA(&huart1,uart1_dma_tx_arry,12);
	//	HAL_ADC_Start(&hadc1);HAL_ADC_Start(&hadc2);
    /* USER CODE END WHILE */
    		if (g_CmdMap[CMD_RESTORE] == 1)
		{
			g_CmdMap[CMD_RESTORE] = 0;
			 memcpy((void*)(&g_CmdMap[CMD_ID]), (void*)(&g_UserCmdMap_Default[0][CMD_ID]), ((27)<<1));		
			Flash_SaveUserCmdmap();
			
		}
		//g_CmdMap[TAG_MOTOR_ENABLE] = 1;
		if (g_CmdMap[CMD_SAVE] == 1) // �Ƿ񱣴��ڴ���Ʊ�
		{
			g_CmdMap[CMD_SAVE] = 0;

			Flash_SaveUserCmdmap();
//			g_RxBufptr = 0;
//			DMA_Cmd(DMA1_Channel6, DISABLE);			
//			DMA1_Channel6->CNDTR = RX_BUF_SIZE;
//			DMA_Cmd(DMA1_Channel6, ENABLE); 
		}
		if (g_CmdMap[SYS_SAVE_TO_FLASH] == 1) // �Ƿ񱣴��ڴ���Ʊ�
		{
			g_CmdMap[SYS_SAVE_TO_FLASH] = 0;			
			Flash_SaveCmdmap();
//			g_RxBufptr = 0;
//			DMA_Cmd(DMA1_Channel6, DISABLE);			
//			DMA1_Channel6->CNDTR = RX_BUF_SIZE;
//			DMA_Cmd(DMA1_Channel6, ENABLE); 
		}	
		
				if (g_CmdMap[CMD_CLEAR_ERROR] == 1) // �Ƿ��������
		{
			clear_flash_erro();
			g_CmdMap[CMD_CLEAR_ERROR] = 0;
			g_CmdMap[CMD_ERROR] = 0;
			ClearError();
		}		
		if (g_CmdMap[SYS_CLEAR_CLIBDATA] == 0x5555) //һ������궨����
		{
			g_CmdMap[SYS_CLEAR_CLIBDATA] = 0;
			pos_linearity_set_default();
		}
		if (g_CmdMap[SYS_SET_FORCE_0N] == 0x0001) //���õ�ǰ����ֵΪ
		{
			g_CmdMap[SYS_YBP_BASE_VALUE_12BIT] = g_CmdMap[CMD_YBP_ADC_12BIT];
			g_CmdMap[SYS_SET_FORCE_0N] = 0x0000;
			Flash_SaveCmdmap();
//			g_RxBufptr = 0;
//			DMA_Cmd(DMA1_Channel6, DISABLE);			
//			DMA1_Channel6->CNDTR = RX_BUF_SIZE;
//			DMA_Cmd(DMA1_Channel6, ENABLE); 
		}
		//Motor_forbidden = 0;
		if(g_CmdMap[CMD_E_STOP] == 1 || g_CmdMap[CMD_PAUSE] == 1 || (g_CmdMap[CMD_ERROR] & 0x0fff) != 0x0000)
		{
			Motor_forbidden = 1;
			g_CmdMap[CMD_PAUSE] = 0;
			Motor_emergercy_stop = 1;
			Motor_pause_flag = 1;
			stop_posWaveSin(&waveGen);
		}

		
		
		UART_CheckReceive();
		UART_CheckReceive_1();
		Rx_Send_Modbus(); //�ظ�modbus
		if(Flag_1ms == 1)//
		{
			Flag_1ms=0;
			user_param_protected();
			detect_limitPos(g_CmdMap[CMD_POS_ACT_PU],g_CmdMap[CMD_CUR_SET_PU]);
			
			if(++s_count_50ms>50)
			{
				s_count_50ms = 0;
				Flag_50ms = 1;
				if(g_CmdMap[SYS_MU_COMM_TO_PC] == 1)     //MU оƬ�궨ģʽ
				{
          g_CmdMap[SYS_MOT_TEST] = 5;

        }	
				else
        {
        }
					
				
			}
			
			sim_ElectricAngle();
			
			if (++s_Count_Rec >= 10)
			{ 
				s_Count_Rec = 0;
				g_SysStatus = 0;															//ϵͳ����״̬
				if(g_CmdMap[TAG_MOTOR_ENABLE] == 1)
				{
					g_SysStatus = g_SysStatus|0x0001;
				}
				if(g_CmdMap[CMD_ERROR] != 0)
				{
					g_SysStatus = g_SysStatus|0x0002;
				}	
			}
		/* USER CODE BEGIN 3 */
		}
		
		if(Flag_50ms == 1)//if (bsp_CheckTimer(0))
		{
			start_temperature_adc();
			Flag_50ms = 0;
			generate_errorCode();
		}
		work_var_updata();  // ���¿��Ʋ���	

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}




/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 1);
//  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 1);
//  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 1);
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 1);
	
	//HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	//HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END 5 */
}
