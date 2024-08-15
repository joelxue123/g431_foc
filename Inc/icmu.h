#ifndef __ICMU_H__
#define __ICMU_H__ 
#include "global.h"


extern SPI_HandleTypeDef hspi1;

#define SPI1_CS_EN     (GPIOA->ODR &= ~GPIO_PIN_4)
#define SPI1_CS_DIS    (GPIOA->ODR |= GPIO_PIN_4)


#define CMD_ACTIVATE		0xB0
#define CMD_SD_TRANS		0xA6   //读传感器数据
#define CMD_SD_STATUS		0xF5
#define CMD_REG_READ		0x97   //读传感器数据
#define CMD_REG_WRITE		0xD2	 //写传感器数据
#define CMD_REG_STATUS  0xAD

#define WRITE_ALL 			0x01
#define WRITE_OFF 			0x02
#define ABS_RESET			  0x03
#define NON_VER 			  0x04
#define MT_RESET 			  0x05
#define MT_VER 				  0x06
#define SOFT_RESET			0x07
#define SOFT_PRES 			0x08
#define SOFT_E2P_PRES   0x09
#define E2P_COM 			  0x0A
#define EVENT_COUNT 		0x0B
#define SWITCH 				  0x0C
#define CRC_VER 			  0x0D
#define CRC_CALC 			  0x0E
#define SET_MTC 			  0x0F
#define RES_MTC 			  0x10
//#define MU150_SPI_BYPASS_EN()     (GPIOB->ODR &= ~GPIO_Pin_9)

#define MU150_SPI_BYPASS_EN()   \
do {     \
  GPIO_InitTypeDef 	GPIO_InitStructure;	    \
	__HAL_SPI_DISABLE(&hspi1);                    \
	GPIO_InitStructure.Pin  = GPIO_PIN_5 | GPIO_PIN_6| GPIO_PIN_7; \
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT; \
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM; \
	GPIO_InitStructure.Pull = GPIO_PULLUP; \
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); \
	GPIO_InitStructure.Pin  = GPIO_PIN_4; \
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT; \
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM; \
	GPIO_InitStructure.Pull = GPIO_PULLUP; \
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);            \
}while(0)
//	SPI_Cmd(SPI1, DISABLE);                    \
//	GPIO_Init(GPIOB, &GPIO_InitStructure);            \





#define MU150_SPI_BYPASS_DIS()    {}
extern s32 MUValue;
extern volatile s32 *p_MU_Value;
extern volatile s32 MU_Value,MU_Value_last;
extern  uint8_t spi1_reg_buf[4];
extern volatile u8 Flag_MU_read;
extern volatile uint8_t spi1_reg[4];
extern u8 spi1_send[4];
uint8_t icmu_spi_init(void);
uint8_t icmu_activiate(uint8_t pactive, uint8_t ractive);
uint32_t icmu_sdtransmission(void);
uint8_t icmu_sdstatus(void);
uint8_t icmu_read_reg(uint8_t addr);
uint8_t icmu_write_reg(uint8_t addr, uint8_t data);
uint8_t icmu_reg_status(void);
//阻塞模式获取 MU数值
int8_t get_MU_Value_block(void);

//void SPI1_MU_Init(void);
void SPI1_Init(void);
void SPI1_MU_DMA_FastRead_Init(void);

void icmu_cmu_mu(uint8_t cmd);
void icmu_sdtransmission_test(void);
 void MX_SPI1_Init(void);
#endif


