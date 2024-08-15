#include "security.h"
#include "global.h"
s32 g_CurrentOverPool = 0;
s32 Over_Temp_Cun = 0;			    // ���¼����ʱ��ʱ��
s32 Low_Voltage_Cun = 0;		    // Ƿѹ�����ʱ��ʱ��
s32 Over_Voltage_Cun = 0;		  // ��ѹ�����ʱ��ʱ��
s16 Power_Avg_tab[8] = {0,0,0,0,0,0,0,0};
s32 Power_Avg_cun = 0;

#define	SHORT_CUR_DEF	((int)(HW_MAX_CURRENT*1.414*1.5))

// ������������·���, 10KHz��ʱ����
uint16_t count_over_current = 0;
uint16_t count_over_temperature = 0;
uint16_t count_Id_error = 0;
uint32_t power_on_time = 0;
float Voltag_f_filter = 0.0f;
float Voltag_f = 0.0f;

static int over_current_staff_event_cnt = 0;

void generate_errorCode(void)
{
		static s32 s_count = 0;
	static s32 temperture_hystersis = 0;
	static s32  count_over_AlerrTemperature = 0;
	
	
	if(++s_count>250)
	{
		s_count = 0;
		g_CmdMap[CMD_ERROR] &= (ERROR_MASK_STALL | ERROR_MASK_OVER_CURRENT);
		
		if( g_CmdMap[CMD_ERROR]  )
		{
			over_current_staff_event_cnt++;
			if( over_current_staff_event_cnt < 3 )
			{
				g_CmdMap[CMD_ERROR] = 0;
			}
			
			if( over_current_staff_event_cnt > 3)
			{
				over_current_staff_event_cnt = 3;
			}
		}
		else
		{
			
		}
		
		
	}
	if(power_on_time<50*5)
	{power_on_time++;}
	
	icmu_eerpom_erro_detect();
	
	if(Flag_error_read_flash == 1)
	{
		g_CmdMap[CMD_ERROR] |= ERROR_MASK_READ_FLASH;
	}
//	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11) == Bit_RESET)
//	{
//		g_CmdMap[CMD_ERROR] |= ERROR_MASK_PREDRIVE_FAULT;
//	}
	if(Flag_error_current_clib == 1)
	{
		g_CmdMap[CMD_ERROR] |= ERROR_MASK_CURRENT_CLIB_FAULT;
	}
	//²¹³ä´úÂë
	if(Flag_error_MU_SPI == 1)//ÐÅºÅÌø±äÒì³£ »òÕß SPIÃ»ÓÐÊÕµ½ÐÅºÅ±¨´Ë¹ÊÕÏ
	{
		g_CmdMap[CMD_ERROR] |= ERROR_MASK_ENCODER_FAULT;
	}
//	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) ==  SET)
//	{	
//		g_CmdMap[CMD_ERROR] |= ERROR_MASK_PREDRIVE_FAULT;
//	}	
//	if(Flag_error_MU_value_instable == 1)
//	{
//		g_CmdMap[CMD_ERROR] |= ERROR_MASK_MU_VALUE_INSTABLE;
//	}
//	if(Flag_error_torsion_dect == 1)
//	{
//		g_CmdMap[CMD_ERROR] |= ERROR_MASK_FORCE_SENSOR_FAULT;
//	}
//	Voltag_f = (float)((int32_t)(ADC3->JDR2)*36300/4096);
//	Voltag_f_filter = 0.8f*Voltag_f_filter+0.2f*Voltag_f;	
//	g_CmdMap[CMD_VOLTAGE] = (int16_t)Voltag_f_filter;//mV	
//	if(power_on_time>(50*5-20))
//	{
//			if(g_CmdMap[CMD_VOLTAGE] > MAX_POWVOL)//¹ýÑ¹
//	{
//			g_CmdMap[CMD_ERROR] |= ERROR_MASK_OVER_VOLTAGE;
//	} 	
//		if(g_CmdMap[CMD_VOLTAGE] < MIN_POWVOL)//Ç·Ñ¹
//		{
//			g_CmdMap[CMD_ERROR] |= ERROR_MASK_UNDER_VOLTAGE;
//		} 
//	}


	if( g_CmdMap[CMD_RUN_LOW_TEMP] > g_CmdMap[CMD_OVER_TEMP_SET])
	{
		g_CmdMap[CMD_RUN_LOW_TEMP] = g_CmdMap[CMD_OVER_TEMP_SET];
	}
	
	if(g_CmdMap[CMD_TEMP] > ( g_CmdMap[CMD_OVER_TEMP_SET] + temperture_hystersis) )
	{
		if(count_over_temperature<30)
		{
			count_over_temperature++;
		}
		else
		{
			count_over_temperature = 31;
			temperture_hystersis = g_CmdMap[CMD_RUN_LOW_TEMP] - g_CmdMap[CMD_OVER_TEMP_SET];

			g_CmdMap[CMD_ERROR] |= ERROR_MASK_OVER_TEMP;
			s_count = 0;
		}		
	}
	else
	{
		temperture_hystersis = 0;
		count_over_temperature = 0;
	}

	if( g_CmdMap[CMD_TEMP] > g_CmdMap[CMD_RUN_LOW_TEMP] )
	{
		if(count_over_AlerrTemperature<30)
		{
			count_over_AlerrTemperature++;
		}
		else
		{
			count_over_AlerrTemperature = 31;
			g_CmdMap[CMD_ERROR] |= ERROR_MASK_HIGH_TEMPERATURE_ALERT; 
			s_count = 0;
		}	
	}
	else
	{
		count_over_AlerrTemperature = 0;
	}
	
	
	
	if(g_CmdMap[CMD_OVER_CURRENT_SET]>16384)
	{
		g_CmdMap[CMD_OVER_CURRENT_SET] = 16384;
	}
	
	if( ABS( (s16)g_CmdMap[CMD_CUR_ACT_PU]) > g_CmdMap[CMD_OVER_CURRENT_SET] )//¹ýÁ÷¹ÊÕÏ
	{
		if(count_over_current<150)
		{
			count_over_current++;		
		}
		else
		{
			count_over_current = 151;
			s_count = 0;
			if( ABS(g_CmdMap[CMD_SPD_ACT_PU]) <  200 )
			{
				
				g_CmdMap[CMD_ERROR] |= ERROR_MASK_STALL;
			}
			g_CmdMap[CMD_ERROR] |= ERROR_MASK_OVER_CURRENT;
			
			
		}		
	}
	else
	{
		count_over_current = 0;
	}	
	
	
	
}
// �������
void ClearError(void)
{
  g_CmdMap[CMD_ERROR] = 0;
	over_current_staff_event_cnt = 0;
	clear_eeprom_erro();
}


// ����LED����ָʾ��״̬��2KHz
void UpdateErrorStateLED(void)
{
  static s32 s_count = 0;
  s32 i = 0;

  if (g_CmdMap[CMD_ERROR] == 0)  // �޴����˳������������
  {
    if (s_count)
    {
      s_count = 0;
//      LEDRED_OFF;
    }
    return;
  }
  for (i=0; i<16; i++)  // �д��󣬼���ǵڼ������󣬲���ֵ��i��LED��500msΪ��������i�Σ���250ms��
  {
    if (g_CmdMap[CMD_ERROR] & (0x0001<<i))
      break;
  }

  s_count++;
  if (s_count >= (i+1)*1000)
  {
//    LEDRED_OFF;
    if (s_count >= (i+3)*1000)
      s_count = 0;
  }
  else if (s_count%500 == 0)
  {
//    if (s_count%1000 == 0)
//     LEDRED_OFF;
//    else
//      LEDRED_ON;
  }
}

//�г̱���
void Stroke_protection(void)
{		
		if(g_CmdMap[TAG_WORK_MODE] == MODE_POSITION)
		{			
			if(g_CmdMap[CMD_POS_ACT_PU]>(g_CmdMap[SYS_POS_UPPER_LIMIT_PU]+ 1638)&& g_CmdMap[CMD_SPDREG_REF_PU]>0)
			{
				g_CmdMap[TAG_MOTOR_ENABLE] = 0; //Debug
			}
			if(g_CmdMap[CMD_POS_ACT_PU]<(g_CmdMap[SYS_POS_LOWER_LIMIT_PU]-819)  && g_CmdMap[CMD_SPDREG_REF_PU]<0)
			{
				g_CmdMap[TAG_MOTOR_ENABLE] = 0; //Debug
			}
		}
}



void motor_fault_check(void)
{
	 static s32 s_count = 0;
	
	
	if(g_CmdMap[CMD_SPD_ACT_PU] <  200 &&  (g_Vq ) > 4095 )
	{
		if( ABS(g_CmdMap[CMD_CUR_ACT_PU]) <  *g_pCur_ref_base_mA /10 )
		{
			s_count++;
			if( s_count > 1000 )
			{
				s_count = 1001;
			//	g_CmdMap[CMD_ERROR] |= ERROR_MASK_MOTOR_ERROR;
			}
		}
		else
		{
			s_count = 0;
		}
	}
}



void user_param_protected(void)
{
	
		if(g_CmdMap[CMD_CUR_LOWER_LIMIT_PU] > 0 )
	{
		g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]  = 0 - g_CmdMap[CMD_CUR_LOWER_LIMIT_PU] ;
	}
	if(g_CmdMap[SYS_CUR_LOWER_LIMIT_PU] > 0 )
	{
		g_CmdMap[SYS_CUR_LOWER_LIMIT_PU]  = 0 - g_CmdMap[SYS_CUR_LOWER_LIMIT_PU];
	}
	

}



