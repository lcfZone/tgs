#ifndef UAPP_AD7768_H_
#define UAPP_AD7768_H_
#include "main.h"

#define AD7768_4_CH  									4
#define AD7768_REG_CH_STANDBY							0x00
#define AD7768_REG_CH_MODE_A							0x01
#define AD7768_REG_CH_MODE_B							0x02
#define AD7768_REG_CH_MODE_SEL							0x03
#define AD7768_REG_PWR_MODE								0x04
#define AD7768_REG_GENERAL_CFG							0x05
#define AD7768_REG_DATA_CTRL							0x06
#define AD7768_REG_INTERFACE_CFG						0x07
#define AD7768_REG_BIST_CTRL							0x08
#define AD7768_REG_DEV_STATUS							0x09
#define AD7768_REG_REV_ID								0x0A
//#define AD7768_REG_DEV_ID_MSB							0x0B
//#define AD7768_REG_DEV_ID_LSB							0x0C
//#define AD7768_REG_SW_REV_ID							0x0D
#define AD7768_REG_GPIO_CTRL							0x0E
#define AD7768_REG_GPIO_WR_DATA							0x0F
#define AD7768_REG_GPIO_RD_DATA							0x10
#define AD7768_REG_PRECHARGE_BUF_1						0x11
#define AD7768_REG_PRECHARGE_BUF_2						0x12
#define AD7768_REG_POS_REF_BUF							0x13
#define AD7768_REG_NEG_REF_BUF							0x14
#define AD7768_REG_CH_OFFSET_1(ch)						(0x1E + (ch) * 3 + ((ch) > 1 ? 0x06 : 0x00))
#define AD7768_REG_CH_OFFSET_2(ch)						(0x1F + (ch) * 3 + ((ch) > 1 ? 0x06 : 0x00))
#define AD7768_REG_CH_OFFSET_3(ch)						(0x20 + (ch) * 3 + ((ch) > 1 ? 0x06 : 0x00))
#define AD7768_REG_CH_GAIN_1(ch)						(0x36 + (ch) * 3 + ((ch) > 1 ? 0x06 : 0x00))
#define AD7768_REG_CH_GAIN_2(ch)						(0x37 + (ch) * 3 + ((ch) > 1 ? 0x06 : 0x00))
#define AD7768_REG_CH_GAIN_3(ch)						(0x38 + (ch) * 3 + ((ch) > 1 ? 0x06 : 0x00))
#define AD7768_REG_CH_SYNC_OFFSET(ch)					(0x4E + (ch) + ((ch) > 1 ? 0x02 : 0x00))
#define AD7768_REG_DIAG_METER_RX						0x56
#define AD7768_REG_DIAG_CTRL							0x57
#define AD7768_REG_DIAG_MOD_DELAY_CTRL					0x58
#define AD7768_REG_DIAG_CHOP_CTRL						0x59

/* AD7768_REG_CH_STANDBY */
#define AD7768_CH_STANDBY(x)							(1 << (x))

/* AD7768_REG_CH_MODE_x */
#define AD7768_CH_MODE_FILTER_TYPE						(1 << 3)
#define AD7768_CH_MODE_DEC_RATE(x)						(((x) & 0x7) << 0)

/* AD7768_REG_CH_MODE_SEL */
#define AD7768_CH_MODE(x)								(1 << (x))

/* AD7768_REG_PWR_MODE */
#define AD7768_PWR_MODE_SLEEP_MODE						(1 << 7)
#define AD7768_PWR_MODE_POWER_MODE(x)					(((x) & 0x3) << 4)
#define AD7768_PWR_MODE_LVDS_ENABLE						(1 << 3)
#define AD7768_PWR_MODE_MCLK_DIV(x)						(((x) & 0x3) << 0)

/* AD7768_REG_DATA_CTRL */
#define AD7768_DATA_CTRL_SPI_SYNC						(1 << 7)
#define AD7768_DATA_CTRL_SINGLE_SHOT_EN					(1 << 4)
#define AD7768_DATA_CTRL_SPI_RESET(x)					(((x) & 0x3) << 0)

/* AD7768_REG_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_CRC_SEL(x)					(((x) & 0x3) << 2)
#define AD7768_INTERFACE_CFG_DCLK_DIV(x)				(((x) & 0x3) << 0)

#define AD7768_RESOLUTION								24

#define	ADC_CS_LOW()									HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET)
#define	ADC_CS_HIGH()									HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET)

#define	ADC_RESET_LOW()        							HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET)
#define	ADC_RESET_HIGH()        						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET)

#define DP4T_ENABLE()                                   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define DP4T_DISABLE()                                   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)

#define DP4T_SHORT() 									do { \
                                                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); \
                                                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); \
																HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); \
															} while(0)

#define DP4T_TWO_POINT_FIVE_VOLTS() 					do { \
                                                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); \
                                                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); \
																HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); \
															} while(0)

#define DP4T_NORMAL() 									do { \
                                                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); \
                                                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); \
																HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); \
															} while(0)

void my_ad7768_init(void);

uint8_t ad7768_read_cmd(uint8_t reg_addr);

uint8_t add7768_write_cmd(uint8_t reg_addr, uint8_t data);

void ad7768_offset_set(uint8_t chn, uint32_t offset);

void ad7768_gain_set(uint8_t chn, uint32_t gain);

void Fill_reg_List(void);

void offsetCalibration(void);

#endif
