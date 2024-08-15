#include "icmu.h"

s32 MUValue = 0;
s32 test_cnt = 0;
uint8_t buf[9] = {0, 0, 0};
u8 spi1_send[4] = {0xA6,0x00,0x00,0x00};
s16 Ang_reg[1024] = {0};
s16 Ang_reg_Index = 0;
s16 Ang_reg_Index_Last = 0;
s16 Ang_reg_Sum = 0;
s16 Ang_reg_Count = 0;
s16 MECode_offset_ICHaus = 0;
uint8_t spi1_reg_buf[4] = {0};
volatile uint8_t spi1_reg[4] = {0};
volatile s32 *p_MU_Value = (s32*)(&spi1_reg[0]);
volatile s32 MU_Value = 0,MU_Value_last = 0;
volatile s32 Speed_T = 0;
volatile s32 Speed_DltAng_pls = 0;
volatile s32 MU_Vlaue_last = 0;
volatile u8 Flag_MU_read = 0;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;


static int Flag_error_read_eeprom = 0;



static void delay_clk(void)
{
  uint16_t i = 1000;
  while(i--)
    __nop();
}





 void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

	HAL_SPI_TransmitReceive_DMA(&hspi1, &spi1_send[0], &spi1_reg_buf[0], 4);
}




#if 1
void SPI1_Init(void)
{
   /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
	SPI1->DR;
  SET_BIT(SPI1->CR2, SPI_RXFIFO_THRESHOLD);
  __HAL_SPI_ENABLE(&hspi1);
}






void SPI_I2S_SendData8(SPI_TypeDef* SPIx, uint8_t Data)
{
  uint32_t spixbase = 0x00;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));

  spixbase = (uint32_t)SPIx; 
  spixbase += 0x0C;
  
  *(__IO uint8_t *) spixbase = Data;
}


uint8_t SPI_I2S_ReceiveData8(SPI_TypeDef* SPIx)
{
  uint32_t spixbase = 0x00;
  
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH_EXT(SPIx));
  
  spixbase = (uint32_t)SPIx; 
  spixbase += 0x0C;
  
  return *(__IO uint8_t *) spixbase;
}


uint8_t icmu_spi_init(void)
{
  uint8_t icmu_reg = 0;  
  SPI1_Init();


  icmu_reg = icmu_read_reg(0x77);
	
	return (1); 
}
static uint16_t SPI_RW(uint16_t data)
{
  /* Loop while DR register in not emplty */
  while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);
  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData8(SPI1, data);
  /* Wait to receive a byte */
  while((__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET));
  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData8(SPI1);  
}
uint8_t icmu_activiate(uint8_t pactive, uint8_t ractive)
{
  SPI1_CS_EN;
  if (CMD_ACTIVATE == SPI_RW(CMD_ACTIVATE))
  {
    if (0x20 == SPI_RW(0x80+ pactive + (ractive<<1)))
    {
      SPI1_CS_DIS;
      return 1;
    }
  }
  SPI1_CS_DIS;
  return 0;
}
uint32_t icmu_sdtransmission(void)
{
  uint32_t temp = 0;			
  SPI1_CS_EN;
  if (CMD_SD_TRANS == SPI_RW(CMD_SD_TRANS))
  {
    buf[0] = SPI_RW(0);
    buf[1] = SPI_RW(0);
    buf[2] = SPI_RW(0);
  }
  temp = buf[0];
  temp = (temp<<8) | buf[1];
  temp = (temp<<8) | buf[2];
  temp = temp>>5;
  SPI1_CS_DIS;
  return  temp;
}

//����ģʽ��ȡ MU��ֵ
int8_t get_MU_Value_block(void)
{
		static s32 count_wait = 0;
		Flag_MU_read = 0;	
		icmu_sdtransmission_test();		
		while(Flag_MU_read == 0)
		{
			if(count_wait>300)//�������ٺ��룬˵��MU��SPIͨѶʧ��
			{
				Flag_error_MU_SPI = 1;
				return 0;
			}
			count_wait++;
			delay_ms(1);			
		}
		return 1;
}
uint8_t icmu_sdstatus(void)
{
  SPI1_CS_EN;
  if (CMD_SD_STATUS == SPI_RW(CMD_SD_STATUS))
  {
    if (SPI_RW(0) == 0x80)
    {
      SPI1_CS_DIS;
      return 1;
    }
    else
    {
      SPI1_CS_DIS;
      return 0;
    }
  }
  return 0;
}

static void delay__ms(int cnt)
{
	volatile int i=0,j=0;
	
	for(i =0 ;i<1000;i++)
		for(j=0;j<cnt;j++);
}
	


uint8_t icmu_read_reg(uint8_t addr)
{
  uint8_t reg = 0;
  uint8_t sta = 0;
	uint8_t i =0;
	
	Flag_error_read_eeprom = 1;
	
	 SPI1_CS_DIS;
	delay__ms(100);
  SPI1_CS_EN;
	delay__ms(10);
	
  if (CMD_REG_READ == SPI_RW(CMD_REG_READ))
  {

    if (addr == SPI_RW(addr))
    {
			do
			{
				SPI1_CS_DIS;
				delay_clk();
				SPI1_CS_EN;
				delay_clk();
				if (CMD_REG_STATUS == SPI_RW(CMD_REG_STATUS))  // if (CMD_REG_STATUS == SPI_RW(CMD_REG_STATUS))
				{
					sta = SPI_RW(0);
					reg = SPI_RW(0);
					if( sta & 0x01)
					{
						if(reg & 0x40)
						{
						}
						else
						{
							Flag_error_read_eeprom =0;
						}
						g_CmdMap[0x12b] = reg;
						break;
					}
				}
				
			}while( i++ < 10);
			
    }
  }
  sta = sta;
	
	delay__ms(100);
 

	


  return reg;
}

uint8_t icmu_write_reg(uint8_t addr, uint8_t data)
{
  SPI1_CS_EN;
  if (CMD_REG_WRITE == SPI_RW(CMD_REG_WRITE))
  {
    if (addr == SPI_RW(addr))
    {
      if (data == SPI_RW(data))
      {
        SPI1_CS_DIS;
        return 1;
      }
    }
  }
  SPI1_CS_DIS;
  return 0;
}

uint8_t icmu_reg_status(void)
{
  SPI1_CS_EN;
  if (CMD_SD_STATUS == SPI_RW(CMD_SD_STATUS))
  {
    if (SPI_RW(0) == 0x80)
    {
      SPI1_CS_DIS;
      return 1;
    }
    else
    {
      SPI1_CS_DIS;
      return 0;
    }
  }
  SPI1_CS_DIS;
  return 0;
}


/*
0x01 WRITE_ALL Write internal configuration and Offset values to EEPROM
0x02 WRITE_OFF Write internal Offset values to EEPROM
0x03 ABS_RESET Reset of Absolute value (including ABZ-part)
0x04 NON_VER Verification of actual position by doing a nonius calculation
0x05 MT_RESET New read in and synchronisation of multiturn value
0x06 MT_VER Read in of multiturn and verification of counted multiturn value
0x07 SOFT_RESET startup with read in of EEPROM
0x08 SOFT_PRES Set output to preset
0x09 SOFT_E2P_PRES Set output to preset and save offset values to EEPROM
0x0A E2P_COM start EEPROM communication
0x0B EVENT_COUNT increment event counter by 1
0x0C SWITCH A variant of WRITE_ALL to write configurations of MODEA and RPL which inhibit register communications
0x0D CRC_VER Verification of CRC16 and CRC8
0x0E CRC_CALC Recalculate internal CRC16 and CRC8 values
0x0F SET_MTC Set MTC-Pin *)
0x10 RES_MTC Reset MTC-Pin *)
0xFF no function
Note: *) MODE_MT=0x00
*/
void icmu_cmu_mu(uint8_t cmd)
{
	

  icmu_write_reg(0x75,cmd);
}

#endif















void DMA1_Channel3_IRQHandler(void)// ��������ж�
{
		if( ( hdma_spi1_tx.DmaBaseAddress->ISR & DMA_FLAG_TC3 ) )
		{
			hdma_spi1_tx.DmaBaseAddress->IFCR |=  DMA_FLAG_TC3;
			__HAL_DMA_DISABLE(&hdma_spi1_tx);
//			SPI1_CS_DIS;
		}
}
s32 error_encoder = 0;

void DMA1_Channel4_IRQHandler(void)// �ɼ���һ�� MU��ֵ�ж�һ�Σ�����ʱ�����������
{
    static int read_cnt = 0; // ÉÏµçÎª0
		if( ( hdma_spi1_rx.DmaBaseAddress->ISR & DMA_FLAG_TC4 ))
		{

      read_cnt++;
			hdma_spi1_rx.DmaBaseAddress->IFCR |=  DMA_FLAG_TC4;

			#if defined(MU_VALUE_INCREASE_REACH)    //�Ƹ����MU��ֵ����
				MU_Value =((((uint32_t)spi1_reg_buf[1])<<16)+(((uint32_t)spi1_reg_buf[2])<<8)+(((uint32_t)spi1_reg_buf[3])<<0));
			#else																		//�Ƹ����MU��ֵ����
				MU_Value =MAX_VALUE_ICMU - ((((uint32_t)spi1_reg_buf[1])<<16)+(((uint32_t)spi1_reg_buf[2])<<8)+(((uint32_t)spi1_reg_buf[3])<<0));	
			#endif
			Flag_MU_read = 1;
			SPI1_CS_DIS;


			if(read_cnt > 1000)
			{
				read_cnt = 1001;
				if( spi1_reg_buf[0] != 0xA6 )
				{
					
					if( g_CmdMap[TAG_WORK_MODE] == MODE_POSITION)
						g_CmdMap[CMD_ERROR] |= ERROR_MASK_LINEAR_MAGNETIC_SENSOR_FAULT;
				}
			}
			if(  spi1_reg_buf[0] == 0xA6 )
			{
			//	read_cnt = 0;
			}

		}
}


void icmu_sdtransmission_test(void)
{
//	SPI1_CS_EN;
//	DMA_SetCurrDataCounter(DMA1_Channel3,4);//DMA2_Stream3->NDTR = 4;	
	__HAL_DMA_DISABLE(&hdma_spi1_tx);
	hdma_spi1_tx.Instance->CNDTR =4;
	__HAL_DMA_ENABLE(&hdma_spi1_tx);
}

void clear_eeprom_erro(void)
{
  Flag_error_read_eeprom =0;

}


void icmu_eerpom_erro_detect(void)
{
	if(Flag_error_read_eeprom)
	{
		g_CmdMap[CMD_ERROR] |= ERROR_MASK_LINEAR_MAGNETIC_SENSOR_FAULT;
	}
}


