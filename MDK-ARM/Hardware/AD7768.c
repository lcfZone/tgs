#include <stdio.h>
#include <stdlib.h>
#include "AD7768.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "debug.h"

uint8_t rxBuffer[512] = {0};
uint8_t reg_code[90];  //总计0x59寄存器
volatile uint8_t receive_flag = 0;

const uint8_t standard_pin_ctrl_mode_sel[3][4] = {
//		MCLK/1,	MCLK/2,	MCLK/4,	MCLK/8
		{0x0,	0x1,	0x2,	0x3},	// Eco
		{0x4,	0x5,	0x6,	0x7},	// Median
		{0x8,	0x9,	0xA,	0xB},	// Fast
};

const uint8_t one_shot_pin_ctrl_mode_sel[3][4] = {
//		MCLK/1,	MCLK/2,	MCLK/4,	MCLK/8
		{0xC,	0xFF,	0xFF,	0xFF},	// Eco
		{0xD,	0xFF,	0xFF,	0xFF},	// Median
		{0xF,	0xE,	0xFF,	0xFF},	// Fast
};


void my_ad7768_init()
{
    ADC_RESET_LOW();
	HAL_Delay(10);
    ADC_RESET_HIGH();
	HAL_Delay(10);
    printf("------start------\r\n");
	Fill_reg_List();
	ADC_CS_HIGH();
	add7768_write_cmd(AD7768_REG_CH_STANDBY,				0x08);		//enable 1通道
	add7768_write_cmd(AD7768_REG_CH_MODE_A,					0x0D);		//默认A Sinc滤波器
																			// x1024 Fast = 8	Khz采样率 0x0D
																			// x512  Fast = 16	Khz采样率 0x0C
																			// x256  Fast = 32	Khz采样率 0x0B
																			// x128  Fast = 64	Khz采样率 0x0A
																			// x64   Fast = 128	Khz采样率 0x09	
																			// x32   Fast = 256	Khz采样率 0x08			
	
	
	
	
	add7768_write_cmd(AD7768_REG_CH_MODE_B,					0x0D);		//默认B Sinc滤波器
	add7768_write_cmd(AD7768_REG_CH_MODE_SEL,				0x00);		//默认所有通道选择A

	add7768_write_cmd(AD7768_REG_PWR_MODE,					0x00);		//bit7			|SLEEP_MODE		：0 Normal operation. 	1 Sleep mode.	
																		//bit[5:4]		|POWER_MODE		：00 Eco mode.  10 Median mode.  11 Fast mode.
																		//bit3		 	|LVDS_ENABLE	：0 LVDS input clock disabled.	 1 LVDS input clock enabled.
																		//bit[1:0] 		|MCLK_DIV		：00 MCLK/32:		10 MCLK/8:		11 MCLK/4:
	
	add7768_write_cmd(AD7768_REG_GENERAL_CFG,				0x22);		//bit5 			|RETIME_EN		：0 Disabled		1 Enable SYNC_OUT signal from MCLK	 
																		//bit4 			|VCM_PD				：0 Enable 			1 VCM Power Down 
																		//blt[1:0]	|VCM_VSEL			：00(AVDD1 - AVSS)/2 V.		01 1.65 V.		10 2.5V		11 2.14V  使用VCM时必须开启通道0

	add7768_write_cmd(AD7768_REG_DATA_CTRL,					0x80);		//bit7			|SPI_SYNC			：0 SPI_SYNC low. 	1 SPI_SYNC high （只有1个设备默认高）
																		//bit4  		|SINGLE_SHOT	：0 Disabled.		1 Enabled.（不开启）
																		//blt[1:0]	|SPI_RESET		：No effect. 

	add7768_write_cmd(AD7768_REG_INTERFACE_CFG,				0x00);	  	//bit[3:2]		|CRC_SELECT		：00 No CRC
																		//blt[1:0]	|DCLK_DIV			：00 分频1/8		01 分频1/4		10 分频1/2		00 不分频
	add7768_write_cmd(AD7768_REG_PRECHARGE_BUF_1,			0xff);
	add7768_write_cmd(AD7768_REG_PRECHARGE_BUF_2,			0xff);	
	add7768_write_cmd(AD7768_REG_POS_REF_BUF,				0x13);		//负极缓冲	
	add7768_write_cmd(AD7768_REG_NEG_REF_BUF,				0x13);		//正极缓冲	
	add7768_write_cmd(AD7768_REG_DIAG_METER_RX,				0x07); 
	add7768_write_cmd(AD7768_REG_DIAG_CTRL,					0x00); 
	for(uint8_t i=0; i<4; i++)
	{
		ad7768_gain_set(i+1, 0x555555);	
	}
	
    printf("------end------\r\n");
	Fill_reg_List();
}


uint8_t ad7768_read_cmd(uint8_t reg_addr)
{
	uint8_t set_buf[2];
	uint8_t read_buf[2];
	set_buf[0] = 0x80 | (reg_addr & 0x7F); 		//reg_addr
	set_buf[1] = 0x00;							//None
	
	ADC_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, set_buf, read_buf, 2, 0xff);
	ADC_CS_HIGH();
	ADC_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, set_buf, read_buf, 2, 0xff);
	ADC_CS_HIGH();
	return read_buf[1];	
}

uint8_t add7768_write_cmd(uint8_t reg_addr, uint8_t data)
{
	uint8_t set_buf[2];
	uint8_t read_buf[2];
	
	set_buf[0] = (reg_addr & 0x7F);						//bit15	write=1
	set_buf[1] = data;						 			//reg data
	
	ADC_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, set_buf, read_buf, 2, 0xff);
	ADC_CS_HIGH();

	return set_buf[1];

}

void ad7768_offset_set(uint8_t chn, uint32_t offset)
{
    add7768_write_cmd(AD7768_REG_CH_OFFSET_1(chn-1), (offset>>16) & 0x000000ff);	//MSB
	add7768_write_cmd(AD7768_REG_CH_OFFSET_2(chn-1), (offset>>8) & 0x000000ff);		//Mid
	add7768_write_cmd(AD7768_REG_CH_OFFSET_3(chn-1), (offset>>0) & 0x000000ff);		//LSB
}


void ad7768_gain_set(uint8_t chn, uint32_t gain)
{
	add7768_write_cmd(AD7768_REG_CH_GAIN_1(chn-1), (gain>>16) & 0x000000ff);	//MSB
	add7768_write_cmd(AD7768_REG_CH_GAIN_2(chn-1), (gain>>8) & 0x000000ff);		//Mid
	add7768_write_cmd(AD7768_REG_CH_GAIN_3(chn-1), (gain>>0) & 0x000000ff);		//LSB
}

void Fill_reg_List()
{
	for(uint8_t i = 0; i < 0x60; i++)
	{
		reg_code[i]=ad7768_read_cmd(i);
		HAL_Delay(10);
        printf("%#x %#x\r\n", i, reg_code[i]);
    }
}

//void Fill_reg_List()
//{
//}

void offsetCalibration(void)
{
  DP4T_SHORT();
  uint8_t ret;
  ret = HAL_SPI_Receive_DMA(&hspi4, rxBuffer, 512);
  int32_t cnt = 0;
  int64_t sum = 0;
  for (;;)
  {
    while (!receive_flag)
    {
      HAL_Delay(1);
    }
    receive_flag = 0;
    cnt++;
    if (cnt > 100)
    {
      int32_t res = (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
      if (res & 0x800000) {
        res |= 0xFF000000;
      }
      sum += (int64_t)res;
      printf("%d %d %d %d %d\r\n", rxBuffer[0], rxBuffer[1], rxBuffer[2], rxBuffer[3], (int32_t)(sum / (cnt - 100)));
    }
    if (cnt == 1000) break;
  }
  int32_t offset = (int32_t)((sum / (1000 - 100)) * 3 / 4);
  printf("offset: %d %d %d %d\r\n", offset, (offset >> 16) & 0xff, (offset >> 8) & 0xff, offset & 0xff);
  ad7768_offset_set(1, offset);
  
  DP4T_DISABLE();
  HAL_Delay(100);
  DP4T_TWO_POINT_FIVE_VOLTS();


  cnt = 0;
  sum = 0;
  for (;;)
  {
    while (!receive_flag)
    {
      HAL_Delay(1);
    }
    receive_flag = 0;
    cnt++;
    if (cnt > 100)
    {
      int32_t res = (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
      sum += (int64_t)res;
      printf("%d %d %d %d %d\r\n", rxBuffer[0], rxBuffer[1], rxBuffer[2], rxBuffer[3], (int32_t)(sum / (cnt - 100)));
    }
    if (cnt == 1000) break;
  }
  double value = ((double)sum) / (1000 - 100);
  uint32_t gain = (uint32_t)(28633110186666.66 / value);
  printf("gain: %u %d %d %d\r\n", gain, (gain >> 16) & 0xff, (gain >> 8) & 0xff, gain & 0xff);
  ad7768_gain_set(1, gain);
  Fill_reg_List();	
  
  DP4T_DISABLE();
  HAL_Delay(100);
  DP4T_SHORT();
}
/* USER HANDLE END 0 */
