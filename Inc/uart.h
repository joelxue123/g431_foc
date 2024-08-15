#ifndef __UART_H__
#define __UART_H__

#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "global.h"


#define DEBUG_Printf		printf

extern UART_HandleTypeDef huart1;

void	self_putchar( char ch);

#define RX_BUF_SIZE		  128 
#define TX_BUF_SIZE		  128 
#define MAX_FRAME_NUM	  4		  // �����е����֡��
#define MAX_BUF_NUM		  128		  // ÿ֡�����С

#define RX_MA_UART		  28 
typedef unsigned short width_t;
#define FRA_ID		1
#define FRA_LENGTH		0
#define FRA_CMD		2
#define FRA_INDEX	3
#define FRA_DATA	5
#define NUM_POINT 4
#define NUM_FRAME 10
#define LENGTH_DATA (4*NUM_POINT)
#define LENGTH_HIGH_FREQ_FRAME (LENGTH_DATA+8)
#pragma pack (2) /*ָ����2�ֽڶ���*/
struct HIGH_FREQ_FRAME{
	u8 frame[NUM_FRAME][LENGTH_HIGH_FREQ_FRAME];
	s16 count;
	s16 ind_head_frame;
	s16 ind_tail_frame;
	s16 num_frame;
	s16 subindex_frame;
	s16 check_sum;	
	s16 internal_us;	
};
#pragma pack ()
extern volatile struct HIGH_FREQ_FRAME struct_hFrame; 
#define UART_TX_OFF()     (GPIOC->ODR &= ~GPIO_Pin_13)//(GPIOB->ODR |= GPIO_Pin_12)
#define UART_TX_EN()    (GPIOC->ODR |= GPIO_Pin_13)
#define SCPE_TXD_LEN	    80      //			// ���ͻ��泤�ȣ�Byte��
extern volatile u8 flag_rev_modbus; 
extern volatile s16 modbus_rly_delay_ms; 
extern u8 g_UserDefined_TxBuf[SCPE_TXD_LEN];	// �Զ������ݷ��ͻ���
extern s32 g_RxBufptr;				      // ��ǰ���ݵ�ַ
extern volatile u8 g_flag_monitor_from_host;   // ��λ���ڼ��
extern volatile u8 g_UARTisSending;	// �������ڷ�������
extern volatile u8 g_fast_debug;
extern s16 g_test_debug;// ��ǰ���ݵ�ַ
extern void UART_Init(void);		      // UART��س�ʼ��		
extern void UART_CheckReceive(void);	// ������ݽ���
extern void UART_CheckReceive_1(void);

extern void UART_CheckSend(void);	   	// ������ݷ���
extern void UART_PushFrame(u8 DataLen, u8 Cmd, u16 Index, u8* pdata); //
extern void UART_PushFrame_Old(u8 DataLen, u8 Cmd, u8 Index, u8* pdata);
extern unsigned int ModBusCRC16(unsigned char *  cmd, unsigned int len);
extern void ModBusCRC16Send(unsigned char *  cmd, unsigned int len);
extern void Rx_Pro_Modbus(u8* p_frame,u8 rx_cur_Num);
extern void ini_HighFreqData(volatile struct HIGH_FREQ_FRAME *ptr);
extern void UART_PushFrame_HighFreqData(volatile struct HIGH_FREQ_FRAME *ptr,s16 set_data,s16 act_data);
extern void UART_SendFrame_HighFreqData(volatile struct HIGH_FREQ_FRAME *ptr);
void Rx_Send_Modbus(void);

#endif
