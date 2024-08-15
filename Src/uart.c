#include "uart.h"


void	self_putchar( char ch)
{
	huart1.Instance->TDR = ch;
	while( __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET );
	
}
	

#if 1

 //   #include <stdio.h>
 //   #include <rt_misc.h>

//    #pragma import(__use_no_semihosting) 

/*----------------------------------------------------------------------------
Write character to Serial Port
#define PRINTF_UART  
*----------------------------------------------------------------------------*/
#define PRINTF_UART USART3

/*----------------------------------------------------------------------------
Read character from Serial Port   (blocking read)
*----------------------------------------------------------------------------*/
UART_HandleTypeDef huart1;

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

uint8_t uart1_dma_rx_arry[128];
uint8_t uart1_dma_tx_arry[128]={0x55,0xaa,0x11,0x22,0x33,0x44,0x55};




#endif
volatile u8 g_flag_monitor_from_host = 0;   // ��λ���ڼ��
volatile u8 g_bUARTPushingFrames = 0;   // ���������д�����

u8 g_UART_RxBuf[RX_BUF_SIZE]; 	// ���ݽ��ջ���
u8 g_UART_Rx_Modbus[RX_BUF_SIZE] = {0};
u16 g_index_Modbus = 0; 
u8 g_UART_TxBuf[TX_BUF_SIZE]; 	// ���ݽ��ջ���
u8 g_UART_Tx_Modbus[TX_BUF_SIZE] = {0}; 	// ���ݽ��ջ���
u8 g_UART_Tx_Modbus_len = 0; 	// ���ݽ��ջ���
s32 g_RxBufptr = 0;	
volatile u16 g_UART_RXlen = 0;// ��ǰ���ݵ�ַ
volatile u8 g_UARTisSending = 0;	// �������ڷ�������
volatile u8 g_fast_debug = 0;	    // 
volatile struct HIGH_FREQ_FRAME struct_hFrame; 
s16 g_test_debug = 0;// ��ǰ���ݵ�ַ
static u8 CheckSum(u8* pdata, u32 len)
{
	u8 sum = 0;
	u32 i;
	for(i=0; i<len; i++)
		sum += pdata[i];
	return sum;
}


/**
  * @brief  ������֡�浽������
  * @param  DataLen: ���ݶγ���
  * @param  Cmd: �������� 
  * @param  Index: �Ĵ�����ַ
  * @param  pdata: ִ�����ڸ��� 
  * @retval None
  */
void UART_PushFrame(u8 DataLen,u8 Cmd, u16 Index, u8* pdata)
{
	s16 i;
	u16 pos = 0;
	g_bUARTPushingFrames = 1;
	g_UART_TxBuf[pos++] = 0xAA;				      // ֡ͷ
	g_UART_TxBuf[pos++] = 0x55;				      // ֡ͷ	
	if (Cmd == CMDTYPE_WR)
	g_UART_TxBuf[pos++] = DataLen+3;		      // ���ݳ���
	else
	g_UART_TxBuf[pos++] = DataLen+3;		      // ���ݳ���
	g_UART_TxBuf[pos++] = g_CmdMap[CMD_ID];	// ID
	g_UART_TxBuf[pos++] = Cmd;
	g_UART_TxBuf[pos++] = Index&0xff;
	g_UART_TxBuf[pos++] = (Index>>8)&0xff;
	for (i=0; i<DataLen; i++)
	{
		g_UART_TxBuf[pos++] = pdata[i];
	}
	g_UART_TxBuf[pos] = CheckSum(&g_UART_TxBuf[2], pos-2);  // У���
	pos++;
	if(g_fast_debug == 0)
	{

		__HAL_DMA_DISABLE(&hdma_usart1_tx);
		hdma_usart1_tx.Instance->CPAR = (uint32_t)(&(USART1->TDR));				
		hdma_usart1_tx.Instance->CNDTR = pos;//g_Frames[g_Frames_Tail].len;
		hdma_usart1_tx.Instance->CMAR = (uint32_t)&g_UART_TxBuf[0];//(uint32_t)&(g_Frames[g_Frames_Tail].buf[0]);
		__HAL_DMA_ENABLE(&hdma_usart1_tx);
//		HAL_UART_Transmit_DMA(&huart1,&g_UART_TxBuf[0],8);
	}		
//	}
	g_bUARTPushingFrames = 0;
}




void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	
	switch (g_CmdMap[CMD_BAUDRATE_UART])
	{
		case BAUD_UART_19200:
			huart1.Init.BaudRate = 19200;
		break;
		case BAUD_UART_57600:
			huart1.Init.BaudRate = 57600;
		break;
		case BAUD_UART_115200:
			huart1.Init.BaudRate = 115200;
		break;
		case BAUD_UART_921600:
			huart1.Init.BaudRate = 921600;
		break;
		default:
			huart1.Init.BaudRate = 115200;
		break;
	}
	
  
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,g_UART_RxBuf,128);
	huart1.Instance->CR3 |= USART_CR3_DMAT;
	
}
/**
  * @brief  ������֡�浽�����У�Ϊ����LA��Э��
  * @param  DataLen: ���ݶγ���
  * @param  Cmd: �������� 
  * @param  Index: �Ĵ�����ַ
  * @param  pdata: ִ�����ڸ��� 
  * @retval None
  */

void UART_PushFrame_Old(u8 DataLen,  u8 Cmd, u8 Index, u8* pdata)
{
	s16 i;
	u16 pos = 0;
	g_bUARTPushingFrames = 1;
	g_UART_TxBuf[pos++] = 0xAA;				      // ֡ͷ
	g_UART_TxBuf[pos++] = 0x55;				      // ֡ͷ	
	g_UART_TxBuf[pos++] = DataLen+2;		    // ���ݳ���
	g_UART_TxBuf[pos++] = g_CmdMap[CMD_ID];	// ID
	g_UART_TxBuf[pos++] = Cmd;
	g_UART_TxBuf[pos++] = Index&0xff;       //������ַ
	for (i=0; i<DataLen; i++)
	{
		g_UART_TxBuf[pos++] = pdata[i];
	}
	g_UART_TxBuf[pos] = CheckSum(&g_UART_TxBuf[2], pos-2);  // У���
	pos++;
	if(g_fast_debug == 0)
	{
				__HAL_DMA_DISABLE(&hdma_usart1_tx);
		hdma_usart1_tx.Instance->CPAR = (uint32_t)(&(USART1->TDR));				
		hdma_usart1_tx.Instance->CNDTR = pos;//g_Frames[g_Frames_Tail].len;
		hdma_usart1_tx.Instance->CMAR = (uint32_t)&g_UART_TxBuf[0];//(uint32_t)&(g_Frames[g_Frames_Tail].buf[0]);
		__HAL_DMA_ENABLE(&hdma_usart1_tx); 
	}		
	g_bUARTPushingFrames = 0;
}
// ����һ֡����
u8 flag_is_writable = 0;

static int last_usr_mode = 0;
void int_last_usr_mode(void)
{
	last_usr_mode = g_CmdMap[CMD_USEER_MODE];
}

void UART_ParseFrame(u8* pdata)
{
	s32 result = 0;
	s32 Index = ((s32)pdata[FRA_INDEX+1]<<8)+(s32)pdata[FRA_INDEX];
	u8* pbWRBuf = &g_CmdMap_bWR[0][0];
	s32 datalen = 0;
	u8* p_source;u8* p_des;
	s16 pos_set_2000;
	s32 i = 0;
//	g_CmdMap[CMD_ID] = 0X01;
	if(pdata[FRA_ID] != g_CmdMap[CMD_ID] && pdata[FRA_ID] != 0xFF)
	{
		return;
	}
	
	g_flag_monitor_from_host = 1;	
	if(pdata[FRA_CMD] == CMDTYPE_WR || pdata[FRA_CMD] == CMDTYPE_WR_NR)
	{
		datalen = (g_UART_RXlen-2)>>1;
		flag_is_writable = 1;		
		if(Index>=USERCMDMAP_LEN)
		{
			pbWRBuf = &g_CmdMap_bWR[0][0];
			if((u16)g_CmdMap[CMD_ACCESS_CODE] != (u16)0x1234)
			{
				flag_is_writable = 0;
			}
			else				
			{
				for(i=0; i<datalen; i++)
				{
					if (*(pbWRBuf+Index-USERCMDMAP_LEN+i) != WRFLG_RW)
					flag_is_writable = 0;			
				}
			}
		}
		else
		{
			pbWRBuf = &g_CmdMap_User_bWR[0][0];
			for(i=0; i<datalen; i++)
			{
				if (*(pbWRBuf+Index+i) != WRFLG_RW)
				flag_is_writable = 0;
			}
		}
		if(flag_is_writable == 1)
		{	
			p_source = (u8*)&pdata[FRA_DATA];
			p_des = (u8*)&g_CmdMap[Index];
			for(i=0;i<((g_UART_RXlen-2-1));i++)
			{
				*(p_des+i) = *(p_source+i);
			}
		}
		//memcpy((s32*)&g_CmdMap[Index], (s32*)&pdata[FRA_DATA], (g_UART_RXlen-2-1));				      
		/* void *memcpy(void*dest, const void *src, size_t n);
		��srcָ���ַΪ��ʼ��ַ������n���ֽڵ����ݸ��Ƶ���destinָ���ַΪ��ʼ��ַ�Ŀռ��� */
		if(pdata[FRA_ID] != BROADCAST_ID && pdata[FRA_CMD] == CMDTYPE_WR)
		{	
			Return_state_frame(CMDTYPE_WR,Index);
		}
		if(Index >= POS_LINEARITY_ADC(0) && Index <= POS_LINEARITY_ADC(NUM_LINEARITY_SEG))	
		{
			pos_linearity_ini();
		}
		if(Index <= CMD_SPD_SET_PU && (Index+datalen) > CMD_SPD_SET_PU && g_CmdMap[TAG_WORK_MODE] == MODE_SPEED )					//�����⵽���͵�Ŀ��λ�ã��ж��Ƿ��˶�			
		{
			if(g_CmdMap[CMD_E_STOP] == 0)
			{
				g_CmdMap[TAG_MOTOR_ENABLE] = 1;
			}
		}
		//λ��+��ģʽ�� ͬʱ�޸� �������ٶȡ�λ�üĴ�������ʼִ��ָ��
		if(Index <= CMD_FORCE_SET_PU && (Index+datalen) >= CMD_POS_SET_PU && (g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE))					
		{
				/*Force>0����£� Pos_set>Pos_act���Ƚ���λ��ģʽ����λ�õ�λ����� ����ģʽ
													Pos_set<Pos_act,ֱ�ӽ��� ����ģʽ
					Force_set<0����£�Pos_set<Pos_act���Ƚ���λ��ģʽ����λ�õ�λ����� ����ģʽ
													Pos_set>Pos_act,ֱ�ӽ��� ����ģʽ
				*/
				if(g_CmdMap[CMD_FORCE_DIR] == 0)
				{
					force_set = g_CmdMap[CMD_FORCE_SET_PU];
					force_act = g_CmdMap[CMD_FORCE_ACT_RAW_PU];
				}
				else
				{
					force_set = -1*g_CmdMap[CMD_FORCE_SET_PU];
					force_act = -1*g_CmdMap[CMD_FORCE_ACT_RAW_PU];
				}

						if(force_set>0) //�������
						{
							if(force_set>force_act) //����δ��λ
							{
								if(g_CmdMap[CMD_POS_SET_PU] >g_CmdMap[CMD_POS_ACT_PU]) //����δ��λ �� λ��Ҳû��λ
								{
									status_Mode_POSITION_FORCE = 1; 
									g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
									recPosCmd();
								}
								else
								{
									status_Mode_POSITION_FORCE = 2; //����δ��λ �� λ�õ�λ  �������ٿ����׶�
									g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;	
								}
							}
							else 
							{
								if(g_CmdMap[CMD_POS_SET_PU] >g_CmdMap[CMD_POS_ACT_PU]) //������λ �� λ��Ҳû��λ ����λ��ģʽ
								{
									status_Mode_POSITION_FORCE = 1; 
									g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
									recPosCmd();
								}
								else    //������λ �� λ�õ�λ  �������ص��ڽ׶�
								{
									status_Mode_POSITION_FORCE = 3; 
									g_CmdMap[TAG_WORK_MODE] = MODE_FORCE;
								}				
							}
						}
						else if(force_set<0)  //�������
						{													
							if(force_set<force_act) //��������λ
							{
								if(g_CmdMap[CMD_POS_SET_PU] <g_CmdMap[CMD_POS_ACT_PU]) //����δ��λ �� λ��Ҳû��λ
								{
									status_Mode_POSITION_FORCE = 1; //λ��+����ģʽ�µ�״̬�� 0 Idle 1 Profile 2 Force_touch  g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE
									g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
									recPosCmd();
								}
								else
								{
									status_Mode_POSITION_FORCE = 2; //��δ��λ �� λ�õ�λ  �������ٿ����׶�
									g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;	
								}
							}
							else 
							{
								if(g_CmdMap[CMD_POS_SET_PU] <g_CmdMap[CMD_POS_ACT_PU]) //����λ �� λ��Ҳû��λ ����λ��ģʽ
								{
									status_Mode_POSITION_FORCE = 1; 
									g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
									recPosCmd();
								}
								else    //����λ �� λ�õ�λ  �������ص��ڽ׶�
								{
									status_Mode_POSITION_FORCE = 3; 
									g_CmdMap[TAG_WORK_MODE] = MODE_FORCE;
								}				
							}							
						}
				else
				{
					status_Mode_POSITION_FORCE = 3; //λ��+����ģʽ�µ�״̬�� 0 Idle 1 Profile 2 Force_touc
				}				
		}		
		if(Index <= CMD_POS_SET_PU && (Index+datalen) > CMD_POS_SET_PU 
			&& (g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_2_SPEED ||g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_0_POSITION||g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_1_SERVO)
			)					//�����⵽���͵�Ŀ��λ�ã��ж��Ƿ��˶�
		{
				g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
				recPosCmd();
		}
		if(Index <= CMD_FORCE_SET_PU && (Index+datalen) >= CMD_FORCE_SET_PU && g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_4_FORCE)//����ģʽ�£�д��������ֱ��ʹ��
		{
				g_CmdMap[TAG_MOTOR_ENABLE] = 1;
				g_CmdMap[TAG_WORK_MODE] = MODE_FORCE;				
		}
		if(Index == SYS_SET_POS_ACT_PU)     
		{
				//���õ�ǰλ��ʱ�������¼��㹫ʽ
			if(g_CmdMap[SYS_MU_COMM_TO_PC] == 1)     //MU оƬ�궨ģʽ
			{
				//	flag_first_exe = 0;
					rod_pos_Muti_Encoder = g_CmdMap[SYS_SET_POS_ACT_PU]*(*g_pPos_ref_base_um)*(int64_t)g_CmdMap[GEAR_RATIO_8BIT]/256/g_CmdMap[SCREW_UM_RES];
			}
			else
				*g_pMU_value_offset = MU_Value;
				range_icmu_vlaue_need_addMax = range_icmu_vlaue_at_stroke - (MAX_VALUE_ICMU - *g_pMU_value_offset);
		}
		if(Index == MOT_EANGLE_OFFSET)	
		{
				g_Encode_offset = (int)g_CmdMap[MOT_EANGLE_OFFSET];
				g_Encode_offset_EN = (g_Encode_offset*EN_360/360);
		}
		if(Index == SYS_SN_PART1)	
		{
			g_CmdMap[CMD_SN_PART1] = g_CmdMap[SYS_SN_PART1];
			g_CmdMap[CMD_SN_PART2] = g_CmdMap[SYS_SN_PART2];
			g_CmdMap[CMD_SN_PART3] = g_CmdMap[SYS_SN_PART3];	
		}	
		if(Index == CMD_PF_TEST_WAVE_F)	
		{
			setPara_waveSinGenrator(&waveGen,g_CmdMap[CMD_PF_TEST_WAVE_F]
																			,g_CmdMap[CMD_PF_TEST_WAVE_PEAK]
																			,g_CmdMap[CMD_PF_TEST_WAVE_TROUGH]
																			,g_CmdMap[CMD_PF_TEST_WAVE_CYCLE]
																			,g_CmdMap[CMD_POS_ACT_PU]);
		}	
		
		if(last_usr_mode != g_CmdMap[CMD_USEER_MODE])
		{
			last_usr_mode = g_CmdMap[CMD_USEER_MODE];
			g_CmdMap[CMD_PAUSE] =1 ;
		}
	}
	else if(pdata[FRA_CMD] == CMDTYPE_GET_STATE)
	{		
				Return_state_frame(CMDTYPE_GET_STATE,0);
	}	
	else if(pdata[FRA_CMD] == CMDTYPE_RD)
	{		
				//UART_PushFrame(pdata[FRA_DATA],CMDTYPE_RD, Index, (u8*)(&g_CmdMap[Index]));	
				UART_PushFrame(2*pdata[FRA_DATA], CMDTYPE_RD, Index, (u8*)(&g_CmdMap[Index]));	//�Ĵ����������ǼĴ����ֽ���
	}
	else if(pdata[FRA_CMD] == CMDTYPE_DEBUG)
	{
				ScopeProDebug();
	}
	else if(pdata[FRA_CMD] == CMDTYPE_DEBUG_2)
	{
				ScopeProDebug_2();
	}
	else if(pdata[FRA_CMD] == CMDTYPE_ADC_CLIB)
	{
				ReturnCLibADC_Frame();
	}
	else if(pdata[FRA_CMD] == CMDTYPE_OLD_RD)//LA��Э��Ķ�ָ��
	{		
		Return_OldFrame_data(CMDTYPE_OLD_RD,pdata[3],pdata[4]);
	}
	else if(pdata[FRA_CMD] == CMDTYPE_OLD_WR)//LA��Э���дָ��
	{
		//ID�������ʡ�����ֵ������ֵ�Ȳ��
		datalen = pdata[0]-2;//д����ֽڳ��� 
		pdata[4]--;//��������
		for(i=0;i<datalen;i++)
		{
			if(pdata[3]+i == 2)//ID
			{
				g_CmdMap[CMD_ID] = pdata[4+i];
			}
			else if(pdata[3]+i == 12)//������
			{
				g_CmdMap[CMD_BAUDRATE_UART] = pdata[4+i];
			}
			else if(pdata[3]+i == 31)//��������У׼λ
			{
				//g_CmdMap[CMD_BAUDRATE_UART] = pdata[4+i];
			}
			else if(pdata[3]+i == 32)//��������ֵ���ֽ�
			{
				g_CmdMap[CMD_OVER_CURRENT_SET] = (pdata[4+i]+(pdata[4+i+1]<<8))/10;
			}
			else if(pdata[3]+i == 98)//���±���ֵ���ֽ�
			{
				g_CmdMap[CMD_OVER_TEMP_SET] = (pdata[4+i]+(pdata[4+i+1]<<8))/10;
			}
			else
			{
			}			
		}
		Return_OldFrame_data(CMDTYPE_OLD_WR,pdata[3],0);
	}
	else if(pdata[FRA_CMD] == CMDTYPE_OLD_CMD )//LA��Э��ĵ���ָ��
	{
		//����
		//��ͣ
		//����
		//��������
		//����λ��ָ��(0-2000)
		if(pdata[4] == 0x04)//ʹ��
		{
//			g_CmdMap[CMD_E_STOP] = 0;
		}
		if(pdata[4] == 0x23)//��ͣ
		{
			g_CmdMap[CMD_E_STOP] = 1;
		}
		if(pdata[4] == 0x23)//��ͣ�˶�
		{
			
		}
		if(pdata[4] == 0x20)//���浽Flash
		{
			g_CmdMap[CMD_SAVE] = 1;
		}
		if(pdata[4] == 0x22)//����������״̬
		{
			
		}
		if(pdata[4] == 0x1E)//�������ָ��
		{
			g_CmdMap[CMD_CLEAR_ERROR] = 1;
		}	
		Return_OldFrame_state(pdata[FRA_CMD],pdata[3],pdata[4]); 
		//Return_OldFrame_data(u8 cmd_type,u8 index,u8 length) 
	}
	else if(pdata[FRA_CMD] == CMDTYPE_OLD_POS || pdata[FRA_CMD] == CMDTYPE_OLD_SERVO_1|| pdata[FRA_CMD] == CMDTYPE_OLD_SERVO_2)//LA��Э��Ķ�λָ�� ���� LA��Э����ŷ�ָ��
	{
		//����λ��ָ��(0-2000)
		if(pdata[3] == 0x37)
		{
			pos_set_2000 = (s16)pdata[4] + (((s16)pdata[5])<<8);
			if(pos_set_2000<0)
			{
				pos_set_2000 = 0;				
			}
			else if(pos_set_2000>2000)
			{
				pos_set_2000 = 2000;
			}
			g_CmdMap[CMD_POS_SET_PU] = pos_set_2000*PU_REFERENCE/2000;
		}
		recPosCmd();
		//Flag_Set_OpenLen = 1;
		g_CmdMap[CMD_SPD_SET_PU] = 16384;
//		pos_interpolation_B_um = pos_interpolation_A_um;
//		pos_interpolation_A_um = g_CmdMap[CMD_POS_SET_PU];
//		time_interpolation_s = (double)g_servo_cmd_period/(double)(F_SPD_REGULATOR_HZ);
//		vel_interpolation_umPs = (s32)((double)(pos_interpolation_A_um-pos_interpolation_B_um)/time_interpolation_s);
//		Profile_ParaSet();
//		if(pdata[FRA_CMD] == CMDTYPE_OLD_SERVO_1|| pdata[FRA_CMD] == CMDTYPE_OLD_SERVO_2)
//		{
//			g_servo_cmd_period = g_servo_cmd_period_cnt;//��Ƶλ���ŷ��£��������� ms
//			g_servo_cmd_period_cnt = 0; //��Ƶλ���ŷ��£��������ڼ�ʱ
//			g_CmdMap[CMD_USEER_MODE] = CMD_USEER_MODE_1_SERVO;
//		}
//		else
//		{
//			g_servo_cmd_period = 3000;//��Ƶλ���ŷ��£��������� ms
//			g_servo_cmd_period_cnt = 0; //��Ƶλ���ŷ��£��������ڼ�ʱ
//			g_CmdMap[CMD_USEER_MODE] = CMD_USEER_MODE_0_POSITION;
//		}		
		Return_OldFrame_state(pdata[FRA_CMD],pdata[3],0); 
	}
	else
	{
		
	}
}

// �������ݵ�Э���ʽ 0x55 0xAA [����]
// ��������
int UART_ParseData(u8 data)
{
	static unsigned char s_RXBuf[RX_BUF_SIZE] = {0};
	static unsigned char s_bFrameHead1 = 0;
	static unsigned char s_bBeginFrame = 0;
	static unsigned char s_rxPos = 0;
	static unsigned char s_rxLen = 0;
	static unsigned char csum = 0;
	int ret = 1;
	
	if(s_bBeginFrame)  // �Ƿ�ʼ�µ�һ֡
	{
		s_RXBuf[s_rxPos] = data;
		if(s_rxPos == FRA_LENGTH)
		{
			s_rxLen = data+3;//data+3;
			if(s_rxLen >= RX_BUF_SIZE)
			{
				s_rxLen  =0;
				s_bFrameHead1 = s_bBeginFrame = 0;
				csum = 0;
				return 1;
			}
			g_UART_RXlen = data;
		}
		s_rxPos++;
		if(s_rxPos == s_rxLen) // һ֡���ݴ������
		{
			if(csum == s_RXBuf[s_rxPos-1])   //���У����ͬ
			{
				UART_ParseFrame(&s_RXBuf[0]);  // ����֡����
				g_index_Modbus = 0;
			}
			else
			{
//				while(1);
			}
			s_bFrameHead1 = s_bBeginFrame = 0;
			s_rxPos = 0;
			csum = 0;
			ret = 0;
			return 0;
		}
		else
		{
			csum += data;
		}
	}
	else if(data == 0x55 && !s_bFrameHead1)
	{
		s_bFrameHead1 = 1;
	}
	else if(data == 0xAA && s_bFrameHead1 && !s_bBeginFrame)
	{
		s_bBeginFrame = 1;
	}
	else
	{
		s_bFrameHead1 = s_bBeginFrame = 0;
		csum = 0;
	}
	
	return 1;
	
	
}

static unsigned char uart_sum(unsigned char *p, int len)
{
	int i = 0;
	unsigned char sum = 0;
	
	for(i=0;i<len;i++)
	{
		sum += p[i];
	}
	return sum;
}

// �������ݵ�Э���ʽ 0x55 0xAA [����]
// ��������
typedef struct
{
	volatile s32 len;
	volatile s32 last_dma_pointer;
  unsigned char buf[RX_BUF_SIZE];
} uart_recv_buf_t;


static uart_recv_buf_t uart_recv_buf = {0,0,{0}};

void UART_CheckReceive(void)
{
	s16 i = 0,len = 0;
  s32 DAMCnt = RX_BUF_SIZE - (DMA1_Channel1->CNDTR);
	s32 cnt_temp = 0,temp =0,rx_len =0,p_rx_len= 0,p_last_len =0,p_last_pointer =0;
	width_t CRC_RX,CRC_Tmp;
	int ret = 1;
	int plen=0;
	int csum =0;
	int j =0;
	unsigned char psum = 0;

	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)//
	{			
		
		//	USART_ClearFlag(USART2,USART_FLAG_IDLE);
		 __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_IDLEF);
		
		
			if(g_bUARTPushingFrames == 1)
			{
//				while(1);
			}
			if( g_RxBufptr != DAMCnt )
			{
				if( DAMCnt > g_RxBufptr )
				{
					len = DAMCnt - g_RxBufptr;
				}
				else
				{
					len = RX_BUF_SIZE + DAMCnt - g_RxBufptr;
				}
			}
			
//			if(len    < 10)
//			{
//				g_RxBufptr = DAMCnt;
//		//		while(1);
//				return;
//			}
//			if(len   >  12)
//			{
//				g_RxBufptr = DAMCnt;
//		//		while(1);
//		//		return;
//			}
			
			temp = (DMA1_Channel1->CNDTR);
			rx_len =  RX_BUF_SIZE - temp; //?????????????,???????????



			if(uart_recv_buf.len < RX_BUF_SIZE - 12)
			{
				p_last_pointer = uart_recv_buf.last_dma_pointer;
				p_rx_len = rx_len - uart_recv_buf.last_dma_pointer;
				
				if(p_rx_len<0)
				{
						p_last_len = RX_BUF_SIZE-uart_recv_buf.last_dma_pointer;
						for(i=0;i<p_last_len;i++)
						{
								uart_recv_buf.buf[uart_recv_buf.len++] = g_UART_RxBuf[p_last_pointer++];
								if(uart_recv_buf.len > RX_BUF_SIZE -1)
								{
										uart_recv_buf.len = 0;
								}
						}
						p_rx_len = rx_len;
						p_last_pointer = 0;
				}
				
				
				for(i=0;i<p_rx_len;i++)
				{
						uart_recv_buf.buf[uart_recv_buf.len++] = g_UART_RxBuf[p_last_pointer++];
						if(uart_recv_buf.len>RX_BUF_SIZE)
						{
								uart_recv_buf.len = 0;
						}
				}
				uart_recv_buf.last_dma_pointer = rx_len;
			}

			
			#if 0
//			while(g_RxBufptr != DAMCnt)
//			{
//				ret = UART_ParseData(g_UART_RxBuf[g_RxBufptr]);
//				if(ret == 0)
//				{
//					g_RxBufptr = DAMCnt;
//					break;
//				}
//				g_UART_Rx_Modbus[g_index_Modbus] = g_UART_RxBuf[g_RxBufptr];
//				g_index_Modbus++;
//				if(g_index_Modbus >= RX_BUF_SIZE)
//				{
//					g_index_Modbus = 0;
//				}
//				cnt_temp++;
//				g_RxBufptr++;
//				if( g_RxBufptr == RX_BUF_SIZE )
//				{
//					g_RxBufptr = 0;
//				}
//			}
			
			#endif
			g_bUARTPushingFrames = 0;
//					USART_ClearFlag(USART2,USART_FLAG_IDLE);
//					DMA_Cmd(DMA1_Channel7, DISABLE);
//					DMA1_Channel7->CPAR = (uint32_t)(&(USART2->TDR));				
//					DMA1_Channel7->CNDTR = g_index_Modbus;
//					DMA1_Channel7->CMAR = (uint32_t)&(g_UART_Rx_Modbus);
//					DMA_Cmd(DMA1_Channel7, ENABLE); 

#if 0
//			if(g_index_Modbus >= 8)
//			{
//										//Ѱ��֡ͷ ID �͹��ܺ�
//										for(i=g_index_Modbus-8;i>0;i--)
//										{
//												if(	(g_UART_Rx_Modbus[i] == g_CmdMap[CMD_ID]|| g_UART_Rx_Modbus[i] ==  0xff)  //ID ��ȷ 
//														&&(g_UART_Rx_Modbus[i+1] == 0x03||g_UART_Rx_Modbus[i+1] == 0x06||g_UART_Rx_Modbus[i+1] == 0x10) //��������ȷ
//													)
//												{
//													break;
//												}											
//										}
////										if(i>g_index_Modbus-3)
////										{
////											g_index_Modbus = 0;
////											return;
////										}
//											CRC_Tmp =  ModBusCRC16(&g_UART_Rx_Modbus[i], g_index_Modbus-2-i); 
//											CRC_RX = ((width_t)g_UART_Rx_Modbus[g_index_Modbus-1]<<8) + g_UART_Rx_Modbus[g_index_Modbus-2];		
//											if(CRC_Tmp == CRC_RX)  //���У�����ͬ
//											{			
//														if(g_UART_Rx_Modbus[i] == g_CmdMap[CMD_ID]|| g_UART_Rx_Modbus[i] ==  0xff)  //ID ��ȷ 
//														{														
//																Rx_Pro_Modbus(&g_UART_Rx_Modbus[i],g_index_Modbus-i);
//														}
//														g_index_Modbus = 0;
//											}
//		}

	#endif
	}
	//Modbus Э�鴦��	
  DAMCnt = RX_BUF_SIZE - (DMA1_Channel1->CNDTR);
	
	


}

void UART_CheckReceive_1(void)
{
	int i = 0;
	int len = 0;
	unsigned char psum = 0;
	int ret = 0;
	width_t CRC_RX,CRC_Tmp;
	
	if( uart_recv_buf.len < 4 )
	{
		uart_recv_buf.len = 0;
	}
	
	
	if(uart_recv_buf.len)
	{
		len = uart_recv_buf.len;
		
//		for(i = 0; i < len; i++)  
//		{
////				g_UART_RXlen = uart_recv_buf.buf[2];
////				UART_ParseFrame(&uart_recv_buf.buf[2]);  // ����֡����
////				g_index_Modbus = 0;
////				uart_recv_buf.len =0;
//				
//				ret = UART_ParseData(uart_recv_buf.buf[i]);
//				if(ret == 0)
//				{
//					uart_recv_buf.len = 0;
//					break;
//				}
//			}
//			
//			if(uart_recv_buf.buf[i] ==0x55 && uart_recv_buf.buf[i+1] ==0xaa  )
//			{
//				plen = uart_recv_buf.buf[i+2];
//				csum =0;
//				
//				for(j=0;j<plen+2;j++)
//				{
//					csum += uart_recv_buf.buf[i+2+j];
//				}
//				
//				if(csum == uart_recv_buf.buf[i+plen+4])   //���У����ͬ
//				{
//					g_UART_RXlen = uart_recv_buf.buf[i+2];
//					UART_ParseFrame(&uart_recv_buf.buf[i+2]);  // ����֡����
//					g_index_Modbus = 0;
//					uart_recv_buf.len = 0;
//					break;
//				}
//			}
//			
//			if( g_RxBufptr == RX_BUF_SIZE )
//			{
//				g_RxBufptr = 0;
//			}
//		}


		for(i = 0; i < len; i++)  
		{
			if(uart_recv_buf.buf[i] ==0x55 && uart_recv_buf.buf[i+1] ==0xaa  )
			{
				if( (i+2) > ( len-1) )
				{
					break;
				}
				g_UART_RXlen = uart_recv_buf.buf[i+2];
				
				if( g_UART_RXlen > (len - i - 5))
				{
					break;
				}
				
				psum = uart_recv_buf.buf[i+g_UART_RXlen+4];
				if( psum == uart_sum(&uart_recv_buf.buf[i+2],g_UART_RXlen+2) )
				{
					UART_ParseFrame(&uart_recv_buf.buf[i+2]);  // ����֡����
					g_index_Modbus = 0;
					uart_recv_buf.len = 0;
					break;
				}
				else
				{
				//	while(1);
				}
			}
		}

		g_index_Modbus = len;
		if(g_index_Modbus >= 8)
		{
										//Ѱ��֡ͷ ID �͹��ܺ�
										for(i=g_index_Modbus-8;i>0;i--)
										{
												if(	(uart_recv_buf.buf[i] == g_CmdMap[CMD_ID]|| uart_recv_buf.buf[i] ==  0xff)  //ID ��ȷ 
														&&(uart_recv_buf.buf[i+1] == 0x03||uart_recv_buf.buf[i+1] == 0x06||uart_recv_buf.buf[i+1] == 0x10) //��������ȷ
													)
												{
													break;
												}											
										}
										if(i>g_index_Modbus-3)
										{
											g_index_Modbus = 0;
											return;
										}
											CRC_Tmp =  ModBusCRC16(&uart_recv_buf.buf[i], g_index_Modbus-2-i); 
											CRC_RX = ((width_t)uart_recv_buf.buf[g_index_Modbus-1]<<8) + uart_recv_buf.buf[g_index_Modbus-2];		
											if(CRC_Tmp == CRC_RX)  //���У�����ͬ
											{			
														if(uart_recv_buf.buf[i] == g_CmdMap[CMD_ID]|| g_UART_Rx_Modbus[i] ==  0xff)  //ID ��ȷ 
														{														
																Rx_Pro_Modbus(&uart_recv_buf.buf[i],g_index_Modbus-i);
														}
														g_index_Modbus = 0;
											}
		}
		uart_recv_buf.len =0;
	}
}
	

void Rx_Pro_Modbus(u8* p_frame,u8 rx_cur_Num)
{
	  int16_t add_reg =0, num_reg = 0,i = 0;
	  s32 result = 1;
		u16 pos = 0;
	  u8* pbWRBuf = &g_CmdMap_bWR[0][0];
//		static int last_cmd = 0;
//		static int cur_cmd = 0;
		g_bUARTPushingFrames = 1;
//		cur_cmd = p_frame[1];
		if(p_frame[1] == 0x03)//��״̬
			 {
				 add_reg = ((int16_t)(p_frame[2]<<8) + p_frame[3]);
				 num_reg = (int16_t)(p_frame[4]<<8) + p_frame[5];
				 g_UART_Tx_Modbus[pos++] = g_CmdMap[CMD_ID];//(uint8_t)*p_ID;
				 g_UART_Tx_Modbus[pos++] = 0x03;
				 g_UART_Tx_Modbus[pos++] = 2*num_reg;//�ֽڳ���
				 for(i=0;i<num_reg;i++)   //�� 2byteѭ����ע��modbus ���� ��λ��ǰ����λ�ں�
				 {
					 g_UART_Tx_Modbus[pos++] = g_CmdMap[add_reg+i] >>8;
					 g_UART_Tx_Modbus[pos++] = g_CmdMap[add_reg+i] & 0x0ff;					 	 
				 }			 				 
			 }
		else if(p_frame[1] == 0x06)//д�Ĵ���
			 {
				 add_reg = ((int16_t)(p_frame[2]<<8) + p_frame[3]);
				 num_reg = 1;
				 for(i=0;i<rx_cur_Num-2;i++)   //�� 2byteѭ����ע��modbus ���� ��λ��ǰ����λ�ں�
				 {
					g_UART_Tx_Modbus[pos++] = p_frame[i];			 
				 }
			 }
		else if(p_frame[1] == 0x10)//д����Ĵ���
			 {
				 add_reg = ((int16_t)(p_frame[2]<<8) + p_frame[3]);
				 num_reg = (int16_t)(p_frame[4]<<8) + p_frame[5];
//				 g_UART_Tx_Modbus[pos++] = g_CmdMap[CMD_ID];    
//				 g_UART_Tx_Modbus[pos++] = 0x10;
//				 g_UART_Tx_Modbus[pos++] = num_reg;
				 for(i=0;i<6;i++)   //�� 2byteѭ����ע��modbus ���� ��λ��ǰ����λ�ں�
				 {
					g_UART_Tx_Modbus[pos++] =    p_frame[i];					 
				 }				 
			}
		else
		{
			g_bUARTPushingFrames = 0;
			return;
		}
	if(p_frame[1] == 0x06 || p_frame[1] == 0x10)
	{		
			{
				if(add_reg>=USERCMDMAP_LEN)
				{
					pbWRBuf = &g_CmdMap_bWR[0][0];
					if((u16)g_CmdMap[CMD_ACCESS_CODE] != (u16)0x1234)
					{
						result = 0;
					}
					else				
					{
						for(i=0; i<num_reg; i++)
						{
						if (*(pbWRBuf+add_reg-USERCMDMAP_LEN+i) != WRFLG_RW)
						result = 0;			
						}
					}
				}
				else
				{
					pbWRBuf = &g_CmdMap_User_bWR[0][0];
					for(i=0; i<num_reg; i++)
					{
						if (*(pbWRBuf+add_reg+i) != WRFLG_RW)
						result = 0;
					}
				}
				if(result)
				{	
					if(p_frame[1] == 0x06)
					{
						g_CmdMap[add_reg] = (p_frame[4]<<8) + p_frame[5];
					}
					if(p_frame[1] == 0x10)
					{
						for(i=0;i<num_reg;i++)  //(g_UART_RXlen-2-1)
						{
							g_CmdMap[add_reg+i] = (p_frame[7+2*i]<<8) + p_frame[8+2*i];
						}
					}
				}
				if(add_reg >= POS_LINEARITY_ADC(0) && add_reg <= POS_LINEARITY_ADC(NUM_LINEARITY_SEG))	
				{
					pos_linearity_ini();
				}				
				if(add_reg <= CMD_POS_SET_PU && (add_reg+num_reg) >= CMD_POS_SET_PU 
				&& (g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_2_SPEED ||g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_0_POSITION||g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_1_SERVO)
				)					//�����⵽���͵�Ŀ��λ�ã��ж��Ƿ��˶�
				{
						g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
						recPosCmd();
				}
				if(add_reg <= CMD_FORCE_SET_PU && (add_reg+num_reg)>= CMD_FORCE_SET_PU && g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_4_FORCE)//����ģʽ�£�д��������ֱ��ʹ��
				{
						g_CmdMap[TAG_MOTOR_ENABLE] = 1;
						g_CmdMap[TAG_WORK_MODE] = MODE_FORCE;				
				}
				
								//λ��+��ģʽ�� ͬʱ�޸� �������ٶȡ�λ�üĴ�������ʼִ��ָ��
				if(add_reg <= CMD_FORCE_SET_PU && (add_reg+num_reg) >= CMD_POS_SET_PU && (g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE))					
				{
						if(g_CmdMap[CMD_FORCE_DIR] == 0)
						{
							force_set = g_CmdMap[CMD_FORCE_SET_PU];
							force_act = g_CmdMap[CMD_FORCE_ACT_RAW_PU];
						}
						else
						{
							force_set = -1*g_CmdMap[CMD_FORCE_SET_PU];
							force_act = -1*g_CmdMap[CMD_FORCE_ACT_RAW_PU];
						}

						if(force_set>0) //�������
						{
							if(force_set>force_act) //����δ��λ
							{
								if(g_CmdMap[CMD_POS_SET_PU] >g_CmdMap[CMD_POS_ACT_PU]) //����δ��λ �� λ��Ҳû��λ
								{
									status_Mode_POSITION_FORCE = 1; 
									g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
									recPosCmd();
								}
								else
								{
									status_Mode_POSITION_FORCE = 2; //����δ��λ �� λ�õ�λ  �������ٿ����׶�
									g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;	
								}
							}
							else 
							{
								if(g_CmdMap[CMD_POS_SET_PU] >g_CmdMap[CMD_POS_ACT_PU]) //������λ �� λ��Ҳû��λ ����λ��ģʽ
								{
									status_Mode_POSITION_FORCE = 1; 
									g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
									recPosCmd();
								}
								else    //������λ �� λ�õ�λ  �������ص��ڽ׶�
								{
									status_Mode_POSITION_FORCE = 3; 
									g_CmdMap[TAG_WORK_MODE] = MODE_FORCE;
								}				
							}
						}
						else if(force_set<0)  //�������
						{													
							if(force_set<force_act) //��������λ
							{
								if(g_CmdMap[CMD_POS_SET_PU] <g_CmdMap[CMD_POS_ACT_PU]) //����δ��λ �� λ��Ҳû��λ
								{
									status_Mode_POSITION_FORCE = 1; //λ��+����ģʽ�µ�״̬�� 0 Idle 1 Profile 2 Force_touch  g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE
									g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
									recPosCmd();
								}
								else
								{
									status_Mode_POSITION_FORCE = 2; //��δ��λ �� λ�õ�λ  �������ٿ����׶�
									g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;	
								}
							}
							else 
							{
								if(g_CmdMap[CMD_POS_SET_PU] <g_CmdMap[CMD_POS_ACT_PU]) //����λ �� λ��Ҳû��λ ����λ��ģʽ
								{
									status_Mode_POSITION_FORCE = 1; 
									g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;
									recPosCmd();
								}
								else    //����λ �� λ�õ�λ  �������ص��ڽ׶�
								{
									status_Mode_POSITION_FORCE = 3; 
									g_CmdMap[TAG_WORK_MODE] = MODE_FORCE;
								}				
							}							
						}
						else
						{
							status_Mode_POSITION_FORCE = 3; //λ��+����ģʽ�µ�״̬�� 0 Idle 1 Profile 2 Force_touc
						}				
				}		
				
				
				
				
				
				if(add_reg == SYS_SET_POS_ACT_PU)     
				{
						//���õ�ǰλ��ʱ�������¼��㹫ʽ
					if(g_CmdMap[SYS_MU_COMM_TO_PC] == 1)     //MU оƬ�궨ģʽ
					{
						//	flag_first_exe = 0;
							rod_pos_Muti_Encoder = g_CmdMap[SYS_SET_POS_ACT_PU]*(*g_pPos_ref_base_um)*(int64_t)g_CmdMap[GEAR_RATIO_8BIT]/256/g_CmdMap[SCREW_UM_RES];
					}
					else
						*g_pMU_value_offset = MU_Value;
						range_icmu_vlaue_need_addMax = range_icmu_vlaue_at_stroke - (MAX_VALUE_ICMU - *g_pMU_value_offset);
				}
				if(add_reg == MOT_EANGLE_OFFSET)	
				{
						g_Encode_offset = (int)g_CmdMap[MOT_EANGLE_OFFSET];
						g_Encode_offset_EN = (g_Encode_offset*EN_360/360);
				}		
			}		
	}
	ModBusCRC16Send(g_UART_Tx_Modbus, pos);//buf
	pos = pos + 2;
	g_UART_Tx_Modbus_len = pos;
	flag_rev_modbus = 1;

}
void Rx_Send_Modbus(void)
{
	if(flag_rev_modbus == 1)
	{
		modbus_rly_delay_ms++;	
	}	
	if(modbus_rly_delay_ms >= 1)
	{
		modbus_rly_delay_ms = 0;
		flag_rev_modbus = 0;
		if(g_fast_debug == 0)
		{
			__HAL_DMA_DISABLE(&hdma_usart1_tx);
			hdma_usart1_tx.Instance->CPAR = (uint32_t)(&(USART1->TDR));				
			hdma_usart1_tx.Instance->CNDTR = g_UART_Tx_Modbus_len;//g_Frames[g_Frames_Tail].len;
			hdma_usart1_tx.Instance->CMAR = (uint32_t)&g_UART_Tx_Modbus[0];//(uint32_t)&(g_Frames[g_Frames_Tail].buf[0]);
			__HAL_DMA_ENABLE(&hdma_usart1_tx);
		}		
		g_bUARTPushingFrames = 0;
	}
}
unsigned int ModBusCRC16(unsigned char *  cmd, unsigned int len)
{
    unsigned int i, j, tmp, CRC16;//out;
    CRC16 = 0xFFFF;             //CRC??????
    for (i = 0; i < len; i++)
    {
        CRC16 ^= cmd[i];
        for (j = 0; j < 8; j++)
        {
            tmp = (unsigned int)(CRC16 & 0x0001);
            CRC16 >>= 1;
            if (tmp == 1)
            {
                CRC16 ^= 0xA001;    //?????
            }
        }
    }
		return CRC16; 
    //cmd[i++] = (byte) (CRC16 & 0x00FF);
    //cmd[i++] = (byte) ((CRC16 & 0xFF00)>>8);
}
//ModbusCRC У��
void ModBusCRC16Send(unsigned char *  cmd, unsigned int len)
{
    unsigned int i, j, tmp, CRC16;//out;
    CRC16 = 0xFFFF;             //CRC??????
    for (i = 0; i < len; i++)
    {
        CRC16 ^= cmd[i];
        for (j = 0; j < 8; j++)
        {
            tmp = (unsigned int)(CRC16 & 0x0001);
            CRC16 >>= 1;
            if (tmp == 1)
            {
                CRC16 ^= 0xA001;    //?????
            }
        }
    }
    cmd[i++] =  (CRC16 & 0x00FF);
    cmd[i++] =  ((CRC16 & 0xFF00)>>8);
}
/**
  * @brief  ������֡�浽������
  * @param  DataLen: ���ݶγ���
  * @param  Cmd: �������� 
  * @param  Index: �Ĵ�����ַ
  * @param  pdata: ִ�����ڸ��� 
  * @retval None
  */
/**
  *   
	* AA 55 length internal cmd cnt_L cnt_H set actL atcH * * * * sum 
  */
void ini_HighFreqData(volatile struct HIGH_FREQ_FRAME *ptr)
{		
		u8 i = 0,j = 0;
		for(i = 0;i<LENGTH_HIGH_FREQ_FRAME-1;i++)
		{
			for(j = 0;j<NUM_FRAME-1;j++)
			{
				ptr->frame[j][i] = 0;
				ptr->frame[j][i] = 0;
			}
		}
		ptr->check_sum = 0;
		ptr->ind_head_frame = 0;
		ptr->ind_tail_frame = 0;
		ptr->num_frame = 0;
		ptr->subindex_frame = 0;
		ptr->count = 0;
		ptr->internal_us = 40; //1000us����
}
/**
  * @brief  ������֡�浽������
  * @param  ptr: ���ݶγ���
  */
void UART_PushFrame_HighFreqData(volatile struct HIGH_FREQ_FRAME *ptr,s16 set_data,s16 act_data)
{
#ifdef HIGHFRE_DEBUG
	u8 i = 0;
	static u16 delay_cnt = 0;
	//������ڷ��Ϳ��У��Ҷ�������֡���ͷ���
	if(ptr->num_frame == 0)
	{
		g_fast_debug = 0;
	}
	if(g_CmdMap[SYS_SELF_TUNING] == 0)
	{
		return;
	}
	if(ptr->num_frame>=NUM_FRAME-1)//�������������������ܼ�����������
	{
		g_CmdMap[SYS_SELF_TUNING] = 0;
		g_CmdMap[TAG_MOTOR_ENABLE] = 0;
		g_CmdMap[CMD_CUR_SET_PU] = 0;
		return;
	}
	if(ptr->ind_head_frame>=NUM_FRAME-1)
	{
		g_CmdMap[SYS_SELF_TUNING] = 0;
		g_CmdMap[TAG_MOTOR_ENABLE] = 0;
		g_CmdMap[CMD_CUR_SET_PU] = 0;
		return;
	}
	if(g_CmdMap[SYS_SELF_TUNING] != 0)
	{
		if(ptr->subindex_frame>=LENGTH_HIGH_FREQ_FRAME-2 || (ptr->num_frame == 0 && ptr->subindex_frame == 0 && ptr->ind_head_frame == 0))
		{
			//���ú÷��ͺ��޸Ĵ�����ݵ�Ŀ�����飬��д��֡ͷ
			if(ptr->subindex_frame == 0 && ptr->ind_head_frame == 0)
			{
				
			}
			else
			{
				ptr->ind_head_frame++;
				ptr->num_frame++;
				if(ptr->ind_head_frame>=NUM_FRAME)
				{
					return;
					ptr->ind_head_frame = 0;
				}
			}
			ptr->subindex_frame = 0;
			ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = 0xAA;				      
			ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = 0x55;				    
			ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = LENGTH_DATA+3;
			ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = ptr->internal_us;//ID
			ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = CMDTYPE_HIGH_F;
			ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = ptr->count&0xff;
			ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = (ptr->count>>8)&0xff;
		}
		ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = set_data&0xFF;
		ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = (set_data>>8)&0xFF;
		ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = act_data & 0xFF;
		ptr->frame[ptr->ind_head_frame][ptr->subindex_frame++] = (act_data>>8)&0xFF;
		ptr->count++;
	}
#endif
}
/**
  * @brief  ������֡�浽������
  * @param  ptr: ���ݶγ���
  */
void UART_SendFrame_HighFreqData(volatile struct HIGH_FREQ_FRAME *ptr)
{
#ifdef	HIGHFRE_DEBUG
	u8 i = 0;
	static u16 delay_cnt = 0;
	//������ڷ��Ϳ��У��Ҷ�������֡���ͷ���
	if(ptr->num_frame == 0)
	{
		g_fast_debug = 0;
	}
	if(ptr->num_frame>0 && g_UARTisSending == 0)
	{
		g_fast_debug = 1;
		if(delay_cnt++>(25*4))
		{
			ptr->check_sum = 0;
			for(i = 2;i<LENGTH_HIGH_FREQ_FRAME-1;i++)
			{
				ptr->check_sum = ptr->check_sum+ptr->frame[ptr->ind_tail_frame][i];
			}
			ptr->frame[ptr->ind_tail_frame][LENGTH_HIGH_FREQ_FRAME-1] = ptr->check_sum;

			DMA_Cmd(DMA1_Channel7, DISABLE);
			DMA1_Channel7->CPAR = (uint32_t)(&(USART2->TDR));				
			DMA1_Channel7->CNDTR = LENGTH_HIGH_FREQ_FRAME;
			DMA1_Channel7->CMAR = (uint32_t)&ptr->frame[ptr->ind_tail_frame][0];
			DMA_Cmd(DMA1_Channel7, ENABLE);
			g_UARTisSending = 1;
			ptr->ind_tail_frame++;
			ptr->num_frame--;
			if(ptr->num_frame == 0)
			{
				g_fast_debug = 0;
			}
			if(ptr->num_frame == 0)
			{
				ptr->check_sum = 0;
				ptr->ind_head_frame = 0;
				ptr->ind_tail_frame = 0;
				ptr->num_frame = 0;
				ptr->subindex_frame = 0;
				ptr->count = 0;
			}
			delay_cnt = 0;
		}
	}
#endif
}



