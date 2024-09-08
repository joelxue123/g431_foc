#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "stm32g4xx_hal.h"
//#include "stm32lib.h"
#include <string.h>
#include <stdio.h>
#include <Math.h>

// �ײ�
#include "uart.h"

#include "flash.h"
#include "pwm.h"
#include "led.h"
#include "nvic.h"

#include "spi_IChaus.h"
#include "spi_MEncoder.h"
#include "adc.h"
// Ӧ�ò�
#include "stm32_math.h"
#include "stm32_math_sin.h"
#include "security.h"
#include "calibration.h"
#include "pid.h"
#include "scope.h"
#include "profile_joint.h"
#include "icmu.h"
#include "performance_test.h"
#include "EM_test.h"

// #define HIGHFRE_DEBUG 0



/*  �г�ѡ��   */
#define DISTANCE_10MM  //10mm �г�
//#define DISTANCE_20MM   //20mm�г�
//#define DISTANCE_30MM   //30mm�г�
/*  ��դ��װ����ѡ��   */

#define MU_VALUE_INCREASE_REACH   //�Ƹ�������̴�դ��ֵ����
//#define MU_VALUE_REDUCE_REACH   //�Ƹ�������̴�դ��ֵ����

/*  ����оƬ��������   */
//#define COAXIAL_DRIVER
#define QIAN_ZHI

/*  �ٱ�����   */
//#define GEAR_8BIT (11<<8)  //�۵���������
//#define GEAR_8BIT (780)		 //ͬ�ᴫ������
#define GEAR_8BIT (10062)  //�۵���������

#define NCT_VOLT 3300
//3300 5700
#if defined(DISTANCE_10MM)
	#define STROKE_UM_RATED   5000       	//�涨�г�
	#define LENGTH_UM_MAGNETIC_GRID   6000 //��դ����
#endif	
#if defined(DISTANCE_20MM)
	#define STROKE_UM_RATED   20000 				//�涨�г�
	#define LENGTH_UM_MAGNETIC_GRID   21000  //��դ����
#endif	
#if defined(DISTANCE_30MM)
	#define STROKE_UM_RATED   30000 				//�涨�г�
	#define LENGTH_UM_MAGNETIC_GRID   33000  //��դ����
#endif


#if defined(COAXIAL_DRIVER)
	#define ADC_CHANNEL_A ADC1
	#define ADC_CHANNEL_C ADC3
	#define PWM_CHANNEL_A CCR3
	#define PWM_CHANNEL_C CCR1
	
	#define OC4_PWM_TIME_OFFSET 1
	#define PWM_OUTPUT_REG   0x5555
	#define MP6540_ON()     (GPIOB->ODR |= GPIO_Pin_12)
	#define MP6540_OFF()    (GPIOB->ODR &= ~GPIO_Pin_12)
#else 
#define OC4_PWM_TIME_OFFSET 1
	#define ADC_CHANNEL_A ADC3
	#define ADC_CHANNEL_C ADC1
	#define PWM_CHANNEL_A CCR1
	#define PWM_CHANNEL_C CCR3
	
	#define PWM_OUTPUT_REG   0x5555
	#define MP6540_ON()     (GPIOB->ODR |= GPIO_PIN_12)
	#define MP6540_OFF()    (GPIOB->ODR &= ~GPIO_PIN_12)
#endif

//#define PWM_OUTPUT_REG   0x5fff 

#define MU_DATA_SPI_RES   24     			//MU SPI���ݷֱ���λ��	
#define MAX_VALUE_ICMU   ((1<<MU_DATA_SPI_RES)-1) //MU SPI�������ֵ
#define DISTANCE_UM_ICMU   48000	 //��դоƬ��Ч�г� um
#define DISTANCE_NM_ICMU   48000000	 //��դоƬ��Ч�г� nm
#define ENABLE_INT()	  __set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	  __set_PRIMASK(1)	/* ��ֹȫ���ж� */
#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

#ifndef true
	#define true  1
#endif

#ifndef false
	#define false 0
#endif
#define PU_REFERENCE         16384            //	
// һ�׵�ͨ�˲�ϵ��
//fL = a/(2*3.14*Ts)
//a  = 6.28*Ts*fL
//Yn = a*Xn+(1-a)*Yn-1
#define		SPEED_FL_HZ	  500.0f
#define		SPEED_FL_A	  (int)(128.0f*6.28f*(1.0f/10000.0f)*SPEED_FL_HZ + 0.5f)

#define		CUR_FL_HZ	  2500.0f
#define		CUR_FL_A	  (int)(256.0f*6.28f*(1.0f/20000.0f)*CUR_FL_HZ + 0.5f)
#define ABS(a)  ((a)<0?-(a):(a))

#define MAX_SPEED_48V_256		  (MAX_SPEED_48V << 8)
#define MAX_ACC					      50000		    // �������ü��ٶ�

#define WRFLG_N			0   // ���ɷ���
#define WRFLG_R			1   // ֻ��
#define WRFLG_RW		2   // �ɶ�д
#define SYS_CLK_MHZ	168  // ϵͳʱ����Ƶ ��λMHz 
#define SVPWM_PERIOD_US	40 //PWM ���� ��λus                                                                             
#define REAL_CCR		  (SVPWM_PERIOD_US*SYS_CLK_MHZ/2)//TIM1 72MHz,1us 72 50us 3600 �������ģʽ������20us(1800)
#define MAX_CCR			  (90*REAL_CCR/100)     //90
#define MAX_CCR_256		(MAX_CCR*256)    			//
#define N05_CCR			  (5*MAX_CCR/100)	 		  // 5%ռ�ձ�
#define N08_CCR			  (8*MAX_CCR/100)		 		// 8%ռ�ձ�
#define N80_CCR			  (80*MAX_CCR/100)		 	// 8%ռ�ձ�
#define N90_CCR			  (90*MAX_CCR/100)	 		// 90%ռ�ձ�
#define N97_CCR			  (97*MAX_CCR/100)	 		// 97%ռ�ձ�
#define TIM_1_8_DEADTIME_CLOCKS (40)
//#define F_CUR_REGULATOR_KHZ	(1000/SVPWM_PERIOD_US)  // ������Ƶ�� ��λKHz,�� PWMִ��Ƶ����ͬ(���õ��߲���)
//#define F_SPD_REGULATOR_KHZ	5  // �ٶȻ�ִ��Ƶ�� ��λKHz
//#define F_POS_REGULATOR_KHZ	1  // λ�û�ִ��Ƶ�� ��λKHz
//#define P_POS_REGULATOR_MS  (1.0/((double)F_POS_REGULATOR_KHZ)) //  λ�ù滮(Profile)ִ�����ڣ���λ�û�ִ��Ƶ����ͬ ��λms

#define F_CUR_REGULATOR_HZ	(1000000/SVPWM_PERIOD_US)  // ������Ƶ�� ��λHz,�� PWMִ��Ƶ����ͬ(���õ��߲���)
#define F_SPD_REGULATOR_HZ	4000  // �ٶȻ�ִ��Ƶ�� ��λHz
#define F_POS_REGULATOR_HZ	4000  // λ�û�ִ��Ƶ�� ��λHz
#define P_POS_REGULATOR_MS  (1000.0/((double)F_POS_REGULATOR_HZ)) //  λ�ù滮(Profile)ִ�����ڣ���λ�û�ִ��Ƶ����ͬ ��λms






#define NOMAL_POWVOL	1000			// �������ѹ
#define MAX_POWVOL		12000			// ��������ѹ
#define MIN_POWVOL		6000			// ��С�����ѹ  �Ž�20180606
#define MAX_TEMP		  750			  // ��߹����¶�

#define 	ENCODE_180	  2048    //�����ͱ�������Ȧ���ۼ�ֵ
#define 	ENCODE_360	  4096    //�����ͱ�������Ȧ���ۼ�ֵ

#define 	ENCODE_SPIMAX	 RES_P_360 //32767   //�ű�SPI�������ֵ ע��65535/2


/*---------------------------------------------------------------���Ʋ���������------------------------------------------------------------------------*/
#ifndef __OUJ_CMD_H__
#define __OUJ_CMD_H__
#define BROADCAST_ID		  0xFF
//ָ�����ͺ궨��
#define CMDTYPE_GET_STATE			  						0x30		//��״̬��Ϣ
#define CMDTYPE_WR			  									0x31		//д���Ʊ�ָ��,������״̬��Ϣ
#define CMDTYPE_RD			  									0x32		//�����Ʊ�ָ��
#define CMDTYPE_WR_NR		  									0x33		//д���Ʊ�ָ��,�޷���
#define CMDTYPE_DEBUG			  								0x38		//�������֡
#define CMDTYPE_DEBUG_2			  							0x3C		//�������֡
#define CMDTYPE_ADC_CLIB                    0x39    //�궨ADC��ֵ����,ʹ�õ�����
#define CMDTYPE_CMD_GETSPILINEARDATA			  0x3A		//��ȡ�ű����Զ�����
#define CMDTYPE_FFT			                    0x3B		//
#define CMDTYPE_HIGH_F			                0x80		//��Ƶ����֡

#define CMDTYPE_OLD_RD			                0x01		//LA��Э��Ķ�ָ��
#define CMDTYPE_OLD_WR			                0x02		//LA��Э���дָ��
#define CMDTYPE_OLD_CMD			                0x04		//LA��Э��ĵ���ָ��
#define CMDTYPE_OLD_POS			                0x21		//LA��Э��Ķ�λָ��
#define CMDTYPE_OLD_SERVO_1			            0x20		//LA��Э����ŷ�ָ��
#define CMDTYPE_OLD_SERVO_2 					      0x22    //


//�ڴ���Ʊ��궨��
#define CMDMAP_INDLEN		  									(34) 		//�ڴ���Ʊ���������
#define CMDMAP_SUBLEN		 										16 		//�ڴ���Ʊ���������
#define CMDMAP_LEN			  									(CMDMAP_INDLEN*CMDMAP_SUBLEN)            //�ڴ���Ʊ��ܳ��ȣ����ֵ�λ��
#define USERCMDMAP_INDLEN		  							16 		//�ڴ���Ʊ���������
#define USERCMDMAP_SUBLEN		 								16 		//�ڴ���Ʊ���������
#define USERCMDMAP_LEN			  							(USERCMDMAP_INDLEN*USERCMDMAP_SUBLEN)	//�ڴ���Ʊ��ܳ��ȣ����ֵ�λ��


#define CMD_USEER_MODE_0_POSITION							0  //��λģʽ
#define CMD_USEER_MODE_1_SERVO			  				1  //�ŷ��岹ģʽ
#define CMD_USEER_MODE_2_SPEED			  				2  //�ٶ�ģʽ
#define CMD_USEER_MODE_3_CURRENT							3  //����ģʽ
#define CMD_USEER_MODE_4_FORCE			  				4  //����ģʽ
#define CMD_USEER_MODE_5_POSITION_FORCE			  5  //λ��+����ģʽ


//������ģʽ����
#define MODE_OPEN			    									0			//����ģʽ
#define MODE_CURRENT		 									 	1			//����ģʽ
#define MODE_SPEED			  									2			//�ٶ�ģʽ
#define MODE_POSITION		  									3			//λ��ģʽ
#define MODE_FORCE		  									  4			//����ģʽ
#define MODE_SPEED_NOCURRENT		  					5			//�ٶȻ� ��������
#define MODE_POSTION_NOCURRENT		  				6			//λ�û�+�ٶȻ� ��������
//�̼��汾��
//#define VERSION_CODE 0x381B     //2021��4��28��
//#define VERSION_CODE 0x3C1E     //2021��12��30��
//#define VERSION_CODE 0x4011     //2022��01��17��
//#define VERSION_CODE 0x4116     //2022��01��21��
//#define VERSION_CODE 0x430F     //2022��03��15��
//#define VERSION_CODE 0x431B     //2022��03��27��
//#define VERSION_CODE 0x431F     //2022��03��31��
//#define VERSION_CODE 0x440E     //2022��03��31��
#define VERSION_CODE 0x5a10     //2022��07��23��
//ϵͳ�����Ĵ���
/*---------------------------------------------------------------���Ʋ��������忪ʼ------------------------------------------------------------------------*/
#define TAG_MOTOR_ENABLE				0x0101		//�����Ƿ��ϵ�
#define TAG_WORK_MODE			      0x0102		//����ģʽ��0-������1-����ģʽ��2-�ٶ�ģʽ��3-λ��ģʽ
#define SYS_SN_PART1 						0x0103		//SN1
#define SYS_SN_PART2 						0x0104		//SN2
#define SYS_SN_PART3			    	0x0105		//SN3
#define SYS_SAVE_TO_FLASH		  	0x0106		//�������ݵ�Flash��־
#define SYS_SET_POS_ACT_PU		  0x0107		//����ʵ��λ�ã�����ʹ�ã������޸��ۼ�ֵ
#define SYS_REC_FORCE_ERROR_CURVE 0x0108  //��¼�����г��ڵļӳ����������
#define SYS_MOTOR_PARA_IDENT			0x0109	//���������ʶʹ�ܣ�1:�������裬2�����Ե��
#define BRAKE_RELEASE_DELAY			0x010A		//��բ�ɿ���ʱ
#define BRAKE_LOCK_DELAY				0x010B		//��բ������ʱ
#define SYS_MU_COMM_TO_PC       0x010C    //��ռ��IC-MUͨѶ��IC-MU����PC��λ�����б궨��        SYS_MOSFET_TEST			  	0x010C		//MOS����
#define SYS_MOT_TEST			    	0x010D		//������� �����������������
#define SYS_SPEED_TEST			  	0x010E		//����ٶȻ�����ģʽ��0-�ٶȻ���������ת��1-�ٶȻ����ڼ���ֵ���������˶���2-�ٶȻ�ģʽ�������¶��˶���3-�ٶȻ�ģʽ����������˶�
#define SYS_CLEAR_CLIBDATA			0x010F		//��ԭ�궨����
//����ֵ
#define SYS_CUR_REF_BASE_MA_L 		0x0110		//�������ۻ�׼ֵmA L_16Bit
#define SYS_CUR_REF_BASE_MA_H 		0x0111		//�������ۻ�׼ֵmA H_16Bit
#define SYS_SPD_REF_BASE_PPS_L 	  0x0112		//����ת�ٱ��ۻ�׼ֵPPS L_16Bit
#define SYS_SPD_REF_BASE_PPS_H 	  0x0113		//����ת���ۻ�׼ֵPPS H_16Bit
#define SYS_SPD_REF_BASE_UMPS_L 	0x0114		//�Ƹ����ٶȱ��ۻ�׼ֵ L_16Bit
#define SYS_SPD_REF_BASE_UMPS_H 	0x0115		//�Ƹ����ٶȱ��ۻ�׼ֵ H_16Bit
#define SYS_POS_REF_BASE_UM_L			0x0116		//λ�ñ��ۻ�׼ֵuM L_16Bit
#define SYS_POS_REF_BASE_UM_H			0x0117		//λ�ñ��ۻ�׼ֵuM H_16Bit
#define SYS_FORCE_REF_BASE_L			0x0118		//����ֵ���ۻ�׼ֵPPS L_16Bit
#define SYS_FORCE_REF_BASE_H			0x0119		//����ֵ�ۻ�׼ֵPPS H_16Bit
#define SYS_ACC_REF_BASE_L				0x011A		//���ٶȱ��ۻ�׼ֵPPS L_16Bit um/s^2
#define SYS_ACC_REF_BASE_H				0x011B		//���ٶȱ��ۻ�׼ֵPPS H_16Bit um/s^2

#define SYS_RESISTANCE_CURPU      0x011D    //ϵͳ�ڲ��ڲ�������Ӧ�ĵ�������ֵ
#define SYS_K_PERCENT_FORCE_CURPU	0x011E		//100% ����� ��Ӧ�ĵ�������ֵ
//��������Ϣ
#define MOT_RES					      0x0120		//�������
#define MOT_INDUC				      0x0121		//������
#define MOT_RATED_VOL			    0x0122		//������ѹ
#define MOT_RATED_CUR			    0x0123		//��������
#define MOT_ENC_LINES			    0x0124		//��������
#define MOT_PAIRS			        0x0125		//���������
#define MOT_EANGLE_OFFSET			0x0126		//�����Ƕ�ƫ����
#define SCREW_UM_RES			    0x0127		//�ݸ��ݾ� ÿȦ��Ӧ���г���um
#define GEAR_RATIO_8BIT	      0x0128		//�����ٱ�*256
#define POSTION_TYPE			    0x0129		//λ�ÿ������� 0.�ݸ��г̿��� 1.�ؽڽǿ��ƣ�2.����
#define SYS_SELF_TUNING			  0x012A		//������������ 1.���������� 2.�ٶȻ����� 

//����Ŀ��ֵ
#define TAG_OPEN_PWM			    0x0130		//����ģʽ��ռ�ձȣ�0~100��
#define TAG_CURRENT_L			    0x0131		//
#define TAG_CURRENT_H			    0x0132		//
#define TAG_SPEED_L				    0x0133		//
#define TAG_SPEED_H				    0x0134		//
#define TAG_POSITION_L			  0x0135		//
#define TAG_POSITION_H			  0x0136		//
#define CMD_CUR_FWD_PU			  0x0137		//ǰ������ֵ
#define CMD_ACC_PRE_PU			  0x0138		//ǰ�������ٶ�
#define ANGLE_GEAR_PASSIVE_MINLEN	0x0139//
#define ANGLE_GEAR_PASSIVE_MAXLEN	0x013A//
#define SYS_MU_MA_0MM_BASE_L	0x013B	//
#define SYS_MU_MA_0MM_BASE_H  0x013C  //
#define SYS_MU_MA_OFFSET_L	  0x013D  //
#define SYS_MU_MA_OFFSET_H	  0x013E  //
#define  SYS_POS_OFFSET         0x013F


//��������ֵ �Լ� YBPƬ����ز���
#define SYS_CUR_UPPER_LIMIT_PU 			0x0140		//������������ ����ֵ
#define SYS_CUR_LOWER_LIMIT_PU 			0x0141		//������������ ����ֵ
#define SYS_SPD_UPPER_LIMIT_PU 			0x0142		//�ٶȿ������� ����ֵ
#define SYS_SPD_LOWER_LIMIT_PU 			0x0143		//�ٶȿ������� ����ֵ
#define SYS_FORCE_UPPER_LIMIT_PU 		0x0144		//���ؿ������� ����ֵ
#define SYS_FORCE_LOWER_LIMIT_PU 		0x0145		//���ؿ������� ����ֵ
#define SYS_POS_UPPER_LIMIT_PU 			0x0146		//λ�ÿ������� ����ֵ
#define SYS_POS_LOWER_LIMIT_PU 			0x0147		//λ�ÿ������� ����ֵ
#define SYS_ACC_LIMIT_PU			    	0x0148		//���ٶȿ��Ƽ��� ����ֵ
#define SYS_SPD_ADD_LIMIT_PU				0x0149		//�����ٶȵ��ڵļ��� ����ֵ
#define CMD_CUR_D_ACT_PU		  			0x014A		//D���������ֵ
#define SYS_YBP_BASE_VALUE_12BIT  	0x014B		//YBP ��׼ֵ
#define SYS_YBP_SLOP_12ADC_FORCEREF 0x014C		//FORCE���ۻ�׼ ��Ӧ��ADC�仯�������� FORCE���ۻ�׼ֵΪ200N������Ĵ�������ҪΪΪ200N��Ӧ��ADC��ֵ�仯��
#define SYS_YBP_ADC_DIR 	        	0x014D		//YBP ���ݱ仯���� ȡ��
#define SYS_SET_FORCE_0N						0x014E    //���õ�ǰ����ֵΪ0N



//���ջ������
#define SEV_PARAME_LOCKED		  		0x0150		//���ջ�����������־
#define SEV_CURRENT_P			    		0x0151		//������P����
#define SEV_CURRENT_I			    		0x0152		//������I����
#define SEV_CURRENT_D			    		0x0153		//������D����
#define SEV_SPEED_P				    		0x0154		//�ٶȻ�P����
#define SEV_SPEED_I				    		0x0155		//�ٶȻ�I����
#define SEV_SPEED_D				    		0x0156		//�ٶȻ�D����
#define SEV_SPEED_DS			    		0x0157		//�ٶ�P����
#define SEV_POSITION_P			  		0x0158		//λ�û�P����
#define SEV_POSITION_I			  		0x0159		//λ�û�I����
#define SEV_POSITION_D			  		0x015a		//λ�û�D����
#define SEV_POSITION_DS			  		0x015b		//λ��P����
#define SEV_FORCE_P				    		0x015c		//���ػ�P����
#define SEV_FORCE_I				    		0x015d		//���ػ�I����
#define SEV_FORCE_D				    		0x015e		//���ػ�D����
#define SEV_FORCE_DS			    		0x015f		//���ػ�����
//״̬��Ϣ֡������
#define SYS_STATA_FRAME_NUM		  0x0160		//״̬��Ϣ֡�����Ĵ����ĸ���
#define SYS_STATA_FRAME_SET(m) (0x0161+m)	//״̬��Ϣ֡��mλ��ŵļĴ�����ַ m:0-15
//̼ĤADC������ϴ��� m:0 - 30
#define POS_LINEARITY_ADC(m)	(0x0170+2*(m))		//̼ĤADC������ϵ㣬ADC��ֵ
#define POS_LINEARITY_PU(m)		(0x0170+2*(m)+1)	//̼ĤADC������ϵ㣬ʵ���г�
#define NUM_LINEARITY_SEG      190  //��Ϸֶε����
#endif
/*---------------------------------------------------------------���Ʋ����������β------------------------------------------------------------------------*/


/*---------------------------------------------------------------�û����������忪ʼ------------------------------------------------------------------------*/
#define CMD_CMD_TYPE 						0x01		//�豸����
#define CMD_FW_VERSION			  	0x02		//�̼��汾
#define CMD_SN_PART1 						0x03		//SN
#define CMD_SN_PART2 						0x04		//
#define CMD_SN_PART3			    	0x05		//
#define CMD_ID					      	0x06		//������ID
#define CMD_BAUDRATE_UART		  	0x07		//���ڲ�����
#define CMD_CLEAR_ERROR					0x08		//�������
#define CMD_E_STOP						  0x09		//��ͣ
#define CMD_PAUSE						    0x0A		//��ͣ����
#define CMD_RESTORE						  0x0B		//��ԭ����
#define CMD_SAVE				        0x0C		//��������
#define CMD_ACCESS_CODE				  0x0D		//Ȩ����֤��
#define CMD_OVER_TEMP_SET			  0x0E		//��������ֵ
#define CMD_RUN_LOW_TEMP			  0x0F		//��������ֵ


#define CMD_OVER_CURRENT_SET			0x10		//��������ֵ
#define CMD_CUR_UPPER_LIMIT_PU 		0x11		//����������
#define CMD_CUR_LOWER_LIMIT_PU 		0x12		//����������
#define CMD_POS_UPPER_LIMIT_PU 		0x13		//�г�����ֵ
#define CMD_POS_LOWER_LIMIT_PU 		0x14		//�г�������
#define CMD_FORCE_DIR 		        0x15		//�������� 0-��ѹΪ�� 1-����Ϊ��

#define CMD_STATE_WORD 		        0x16		//Çý¶¯Æ÷×´Ì¬×Ö ÔËÐÐ¡¢¹ÊÕÏ¡¢ÔÚÉÏ¼«ÏÞ¡¢ÔÚÏÂ¼«ÏÞ
#define CMD_LIMIT_UPPER 		      0x17		//Çý¶¯Æ÷ÉÏ¼«ÏÞ 
#define CMD_LIMIT_LOWER 		      0x18		//Çý¶¯Æ÷ÏÂ¼«ÏÞ
#define CMD_LIMIT_DETECT_ENABLE					0x19


#define CMD_ACC_SET_PU			      0x1A		//���ٶ����ñ���ֵ
#define CMD_USEER_MODE 		        0x20		//�û�����ģʽ,0-��λ��1-�ŷ�,2-�ٶ�,3-����,4-����,5-�ٶ�����ģʽ
#define CMD_CUR_SET_PU			      0x21		//�������ñ���ֵ
#define CMD_FORCE_SET_PU			    0x22		//��������,�Ƿ�߱���������
#define CMD_SPD_SET_PU			      0x23		//�ٶ����ñ���ֵ
#define CMD_POS_SET_PU			  	  0x24		//Ŀ��λ�ñ���ֵ
#define CMD_SPD_SOFT_SET_PU			  0x25		//���Ӵ�ʱ���ٶ�ֵ

#define CMD_POS_ACT_PU		  		  0x26		//ʵ��λ�ñ���ֵ
#define CMD_CUR_ACT_PU		  			0x27		//ʵ�ʵ�������ֵ
#define CMD_SPD_ACT_PU					  0x28		//ʵ���ٶȱ���ֵ
#define CMD_FORCE_ACT_PU					0x29		//ʵ������ֵ
#define CMD_ERROR			        		0x2A		//������
#define CMD_TEMP			        		0x2B		//�����¶�
#define CMD_VOLTAGE				        0x2C		//��ǰ�����ѹ
#define CMD_FORCE_ACT_RAW_PU			0x2D		//ʵ������δ�˲�
#define CMD_SPDREG_REF_PU				  0x2E		//�ٶȻ����ڸ���ֵ
#define CMD_SPD_ADD_PU					  0x2F		//�ٶȸ���ֵ�ı���ֵ
#define CMD_PROFILE_POS_PU				0x30		//���η�����λ���������ֵ
#define CMD_PROFILE_SPD_PU				0x31		//���η������ٶ��������ֵ
#define CMD_POS_ADC_12BIT				  0x32		//λ�ô�����ԭʼֵ
#define CMD_POS_ACT_PU_ADC      	0x33		//λ�ô������˲�ֵ
#define CMD_YBP_ADC_12BIT      	  0x34		//YBPԭʼADC��ֵ






#define CMD_PF_TEST_WAVE_F      	  0x50		//���Ҳ�����Ƶ��
#define CMD_PF_TEST_WAVE_CYCLE      0x51		//���Ҳ���������
#define CMD_PF_TEST_WAVE_PEAK      	0x52		//���Ҳ���������
#define CMD_PF_TEST_WAVE_TROUGH     0x53		//���Ҳ���������
#define CMD_JTEST_TIME1             0x60		//ת���������Լ���ʱ��
#define CMD_JTEST_TIME2             0x61		//ת���������Լ���ʱ��

#define CMD_MOD1_IO_POS      	  		0x70		//MOD1_OI POS
#define CMD_MOD1_IO_SPD      	  		0x71		//MOD1_OI SPD
#define CMD_MOD1_IO_FORCE      	  	0x72		//MOD1_OI FORCE
#define CMD_MOD2_IO_POS      	  		0x73		//MOD2_OI POS
#define CMD_MOD2_IO_SPD      	  		0x74		//MOD2_OI SPD
#define CMD_MOD2_IO_FORCE      	  	0x75		//MOD2_OI FORCE
#define CMD_MOD3_IO_POS      	  		0x76		//MOD3_OI POS
#define CMD_MOD3_IO_SPD      	  		0x77		//MOD3_OI SPD
#define CMD_MOD3_IO_FORCE      	  	0x78		//MOD3_OI FORCE
#define CMD_MOD4_IO_POS      	  		0x79		//MOD4_OI POS
#define CMD_MOD4_IO_SPD      	  		0x7A		//MOD4_OI SPD
#define CMD_MOD4_IO_FORCE      	  	0x7B		//MOD4_OI FORCE
/*---------------------------------------------------------------�û����������忪ʼ------------------------------------------------------------------------*/
//�����ʺ궨��
#define BAUD_UART_19200		  0x0000		//
#define BAUD_UART_57600	    0x0001		//
#define BAUD_UART_115200	  0x0002		//
#define BAUD_UART_921600	  0x0003		//
#define BAUD_CAN_250000		  0x0000		//250K
#define BAUD_CAN_500000		  0x0001		//500K
#define BAUD_CAN_1000000	  0x0002		//1M
//�����ֽ�MASK����
#define ERROR_MASK_STALL			        0x0001		//��ת
#define ERROR_MASK_OVER_TEMP					0x0002		//����
#define ERROR_MASK_OVER_CURRENT				0x0004		//����
#define ERROR_MASK_MOTOR_ERROR				0x0008		//�����쳣
#define ERROR_MASK_READ_FLASH					0x0010    //��ȡFlash�쳣
#define ERROR_MASK_PREDRIVE_FAULT			0x0020		//Ԥ��������
#define ERROR_MASK_ENCODER_FAULT	    0x0040		//�������쳣
#define ERROR_MASK_CURRENT_CLIB_FAULT	0x0080		//����δУ׼
#define ERROR_MASK_OVER_VOLTAGE				0x0100		//��ѹ
#define ERROR_MASK_UNDER_VOLTAGE			0x0200		//Ƿѹ
#define ERROR_MASK_LINEAR_MAGNETIC_SENSOR_FAULT			0x0800		//ÏßÐÔ´ÅÕ¤´«¸ÐÆ÷±¨´í
#define ERROR_MASK_HIGH_TEMPERATURE_ALERT 0x8000


//ȫ�ֱ���
extern volatile s32 pos_nm_MU150;
extern volatile u8 Motor_forbidden;     		// �����ֹ�����־
extern volatile u8 Motor_emergercy_stop;
extern volatile u8 Motor_pause_flag;
extern volatile s16 Zero_s16;         			// SPI����CLK�ź�ʹ��
extern volatile u8 g_BeginPIDFlag;		      // ��ʼPID
extern volatile s16 g_SysStatus; 		      	// ������״̬��
extern volatile s32 g_Timer1CCR;		        // ��ʱ�����յ����ռ�ձ�
extern volatile s16 g_bEnableHB;
extern s16 g_Temperature; 			          	// �¶�
extern u16 g_PowVoltage;			          		// �����ѹ
extern volatile s32 g_MaxCur;	  						// ������
extern volatile s32 g_VelAddMAX;						// ��󸽼��ٶ�
extern volatile s32 g_MaxSpd; 							// ����ٶ�
extern s32 Encode_PPR;
extern s32 Encode_PP_Half_R;
extern double Spd_ratio;
extern uint8_t Flag_error_read_flash; 		//flash ��ȡ����
extern uint8_t Flag_error_set_preDrive;	//����Ԥ����оƬ��������
extern uint8_t Flag_error_sensor_comm;		//������ģ��ͨѶ����
extern uint8_t Flag_error_current_clib;	//����У׼ʧ��
extern uint8_t Flag_error_MU_SPI;				//MU SPIͨѶ����
extern uint8_t Flag_error_MU_value_instable;		//MU �źŲ��ȶ�
extern uint8_t Flag_error_torsion_dect;	//�α��������󣬸��ض˺͵���˵ĽǶ�ƫ��̫��

extern s32 range_icmu_vlaue_at_stroke;
extern s32 vlaue_icmu_offset_overturn;
extern s32 range_icmu_vlaue_need_addMax;

/*-------------------------Profile-----------------------------------*/
extern s32* g_pMU_value_offset;
extern s32* g_pMU_0mm_offset;
extern float pos_p_vel_float;
extern s32* g_pPos_ref_base_um;//λ�ñ���ֵ
extern s32* g_pVel_ref_base_PPS;//�ٶȱ���ֵ
extern s32* g_pVel_ref_base_umPS;//�ٶȱ���ֵ
extern s32* g_pCur_ref_base_mA;//��������ֵ
extern s32* g_pAcc_ref_base_mmPS2;//���ٶȱ���ֵ
extern s32* g_pForce_ref_base_mg;//��������ֵ
extern s32  g_Vel_Max_PosPU_pSec; //������ٶ� pu(λ��) /s
extern s32 g_servo_cmd_period_cnt; //��Ƶλ���ŷ��£��������ڼ�ʱ
extern s32 g_servo_cmd_period; //��Ƶλ���ŷ��£���������
extern s32 g_servo_cmd_pos_cur;//��Ƶλ���ŷ��£���ǰ��λ���趨
extern s32 g_servo_cmd_pos_last;//��Ƶλ���ŷ��£���һ���ڵ�λ���趨
extern s32 pos_set_current_frm_CMD_PU;
extern s32 pos_set_last_frm_CMD_PU;
extern u8 status_Mode_POSITION_FORCE; //λ��+����ģʽ�µ�״̬�� 0 Idle 1 Profile 2 Force_touch
/*-------------------------λ�û�-----------------------------------*/
extern s32 rod_pos_Muti_Encoder;							  // ��������ֵ�ۼ�ֵ������Ȧ���Ȧ�������
extern s32 gPos_Kp;															// ��ǰ��ʵʱ��У׼���λ�û�����ϵ��
extern s32 gPos_Ki;															// ��ǰ��ʵʱ��У׼���λ�û�����ϵ��
//s32 gPos_Kd;    
extern s32 gPos_ds;      												// ��ǰ��ʵʱ��У׼���λ�û�����
//extern s32 gPos_RegOut;  												// λ�û����������(Cnt/s)
//extern s32 gPos_RegMax;  												// λ�û�����������޸�(Cnt/s)
////extern s32* g_pPos;					  									// ��ǰλ��32λָ��(�ؽڽ�)
//extern s32* g_pAngle;					  								// ��ǰλ��32λָ��(�ؽڽ�)
//extern s32* g_pAngle_offset;					  				// ��ǰλ��32λָ��(�ؽڽ�)ƫ����
//extern s32* g_pAngle_sensor;					  				// �Ƕȴ�����ԭʼֵ
//extern s32* g_pTagAngle;					  						// ��ǰλ��32λָ��(�ؽڽ�)
//extern s32* g_pTagPos_nm;												// Ŀ��λ��32λָ��(�ؽڽ�)
//extern s32  g_TagPos_nm_Last;
//extern s32* g_pPos_Rod;													// Ŀ��λ��32λָ��(�Ƹ��г�)
//extern s32* g_pPos_nm;											    // �г̣���λnm  //CMD_OPENLEN_ACT_L
extern volatile u8 g_PosReg_runing;		      		// λ�û�״̬ 1����0ֹͣ
extern volatile u8 g_PosReg_Enbale;		      		// λ�û�ʹ������
extern volatile u8 g_PosReg_Disabel;		      	// λ�û���������

/*-------------------------�ٶȻ�-----------------------------------*/
extern s32 gSpd_Kp;															// ��ǰ��ʵʱ��У׼����ٶȻ�����ϵ��
extern s32 gSpd_Ki;															// ��ǰ��ʵʱ��У׼����ٶȻ�����ϵ��
//s32 gSpd_Kd;	
extern double gVel_PPS;
extern double Vel_PPS_raw;
extern s32 gSpd_ds;      												// ��ǰ��ʵʱ��У׼����ٶȻ�����


extern volatile u8 g_SpdReg_runing;		      		// �ٶȻ�״̬ 1����0ֹͣ
extern volatile u8 g_SpdReg_Enbale;		      		// �ٶȻ�ʹ������
extern volatile u8 g_SpdReg_Disabel;		      	// �ٶȻ���������

/*-------------------------������-----------------------------------*/
extern s32 gCur_Kp;															// ��ǰ��ʵʱ��У׼��ĵ���������ϵ��
extern s32 gCur_Ki;															// ��ǰ��ʵʱ��У׼��ĵ���������ϵ��
//s32 gCur_Kd;	
extern volatile u8 g_CurReg_runing;		      		// �ٶȻ�״̬ 1����0ֹͣ
extern volatile u8 g_CurReg_Enbale;		      		// �ٶȻ�ʹ������
extern volatile u8 g_CurReg_Disabel;		      


extern volatile s16 force_set,force_act;

/*---------------------------�ű�Ƕ����---------------------------------*/
extern volatile s32 g_ElectricAngle_act;	    // ��ʵ��Ƕ�
extern volatile s32 g_ElectricAngle_sim;	    // �����Ƕ�
extern volatile s32 g_ElectricAngle; 	    			// ��ǰ��Ƕ�
extern volatile s32 g_ElectricAngle_15bit;// ��Ƕ�_32768
extern volatile s32 g_ElectricAngle_15bit_Raw;
extern volatile s32 g_MechanicsAngle_15bit;// ��е�Ƕ�_32768
extern volatile s32 g_MechanicsAngle_15bit_last;	// ��е�Ƕ�_32768
extern volatile s32 g_ElectricAngle; 	    			// ��ǰ��Ƕ�

extern volatile s16 state_Jtest;
extern volatile u8 flag_processing_Jtest;

extern s16	EN_0;
extern s16	EN_30;
extern s16	EN_45;
extern s16	EN_60;
extern s16	EN_90;
extern s16	EN_120;
extern s16	EN_150;
extern s16	EN_180;
extern s16	EN_210;
extern s16	EN_240;
extern s16	EN_270;
extern s16	EN_300;
extern s16	EN_330;
extern s16	EN_360;
//
extern volatile s32 L_Slop_Array[NUM_LINEARITY_SEG+1];
//
extern s32	Screw_um;           //�ݾ�
extern s32	Mot_Pairs;          //���������

extern s16 s_count_50ms;
extern s16 s_count_5ms;
extern s16 s_count_1ms;
extern u8 Flag_1ms;
extern u16 Flag_50ms;

// �����궨����
extern s32	g_ZeroCur_MotorA;    // ��������궨ֵ
extern s32	g_ZeroCur_MotorB;    // ��������궨ֵ
extern s32	g_ZeroCur_MotorC;	   // ��������궨ֵ

extern s16 Gripper_Move;
extern u8 Flag_contact;
extern u8 Flag_grip_F_OK;
extern u8 Flag_grip_P_OK;
extern u8 Flag_rellease_P_OK;

// �ڴ���Ʊ�32λ����ָ��
extern s32 g_PosOffset;						// �궨ƫ����
extern s32 g_PosSet_Calibrate;

//�ڴ���Ʊ�
extern s16 g_ThreeLoopParaBefLock[CMDMAP_SUBLEN];
extern volatile s16 g_CmdMap[CMDMAP_LEN+USERCMDMAP_LEN];
//extern volatile s16 ClibData_offset[2048];
extern s16 g_CmdMap_Default[CMDMAP_INDLEN][CMDMAP_SUBLEN];
extern s16 g_UserCmdMap_Default[USERCMDMAP_INDLEN][USERCMDMAP_SUBLEN]; 
//�ڴ���Ʊ���дȨ��
extern u8 g_CmdMap_bWR[CMDMAP_INDLEN][CMDMAP_SUBLEN];
extern u8 g_CmdMap_User_bWR[USERCMDMAP_INDLEN][USERCMDMAP_SUBLEN];
/* �ṩ������C�ļ����õĺ��� */
extern void System_RunPer10ms(void);
extern void System_RunPer1ms(void);
extern void System_Idle(void);


extern s16 g_Sector_test; 			        
extern s16 g_AngleAtSector;
extern s16  g_tk ;
extern s16  g_tk_1;
extern s16  g_t0 ;
extern float bus_voltage_;
extern struct WAVE_SIN_GEN_TYPE waveGen;

void get_average_force_data(void);


#endif

