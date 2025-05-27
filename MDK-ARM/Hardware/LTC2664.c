#include <stdint.h>
#include <math.h>
#include "LTC2664.h"
#include <SPI.h>
#include "debug.h"

const float LTC2664_MIN_OUTPUT[5] = {0.0, 0.0, -2*2.5, -4*2.5, -2.5};
const float LTC2664_MAX_OUTPUT[5] = {2*2.5, 4*2.5, 2*2.5, 4*2.5, 2.5};

int8_t LTC2664_write(uint8_t dac_command, uint8_t dac_address, uint16_t dac_code)
// Write the 16-bit dac_code to the LTC2664
{
  static uint8_t last_data_array[3];
  uint8_t data_array[4], rx_array[4];
  int8_t ret;
  LT_union_int16_2bytes data;
  
  dac_code &= 0x0FFF;
  dac_code <<= 4;
  data.LT_int16 = dac_code;                              // Copy DAC code to union
  data_array[0] = 0;
  data_array[1] = dac_command | dac_address;             // Build command / address byte
  data_array[2] = data.bytes.LT_byte[1];                       // MS Byte
  data_array[3] = data.bytes.LT_byte[0];                       // LS Byte
  printf("dac_code:%u\r\nLT_byte[0]:%u\r\nLT_byte[1]:%u\r\n\r\n", dac_code, data.bytes.LT_byte[0], data.bytes.LT_byte[1]);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi6, data_array, rx_array, 4, 0xff);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
  // Compare data read back to data that was sent the previous time this function was called

  last_data_array[0] = data_array[1]; // Copy data array to a static array to compare
  last_data_array[1] = data_array[2]; // the next time the function is called
  last_data_array[2] = data_array[3];
  printf("rx_array[1]:%u\r\nrx_array[2]:%u\r\nrx_array[3]:%u\r\nlast_data_array[0]:%u\r\nlast_data_array[1]:%u\r\nlast_data_array[2]:%u\r\n\r\n", rx_array[1], rx_array[2], rx_array[3], last_data_array[0], last_data_array[1], last_data_array[2]);
  return !((rx_array[3] == last_data_array[2]) && (rx_array[2] == last_data_array[1]) && (rx_array[1] == last_data_array[0]));
}

int8_t LTC2664_write_command(uint8_t dac_command, uint8_t dac_address, uint16_t dac_code)
// Write the 16-bit dac_code to the LTC2664
{
  static uint8_t last_data_array[3];
  uint8_t data_array[4], rx_array[4];
  int8_t ret;
  LT_union_int16_2bytes data;
  data.LT_int16 = dac_code;                              // Copy DAC code to union
  data_array[0] = 0;
  data_array[1] = dac_command | dac_address;             // Build command / address byte
  data_array[2] = data.bytes.LT_byte[1];                       // MS Byte
  data_array[3] = data.bytes.LT_byte[0];                       // LS Byte
  printf("dac_code:%u\r\nLT_byte[0]:%u\r\nLT_byte[1]:%u\r\n\r\n", dac_code, data.bytes.LT_byte[0], data.bytes.LT_byte[1]);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi6, data_array, rx_array, 4, 0xff);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
  // Compare data read back to data that was sent the previous time this function was called

  last_data_array[0] = data_array[1]; // Copy data array to a static array to compare
  last_data_array[1] = data_array[2]; // the next time the function is called
  last_data_array[2] = data_array[3];
  printf("rx_array[1]:%u\r\nrx_array[2]:%u\r\nrx_array[3]:%u\r\nlast_data_array[0]:%u\r\nlast_data_array[1]:%u\r\nlast_data_array[2]:%u\r\n\r\n", rx_array[1], rx_array[2], rx_array[3], last_data_array[0], last_data_array[1], last_data_array[2]);
  return !((rx_array[3] == last_data_array[2]) && (rx_array[2] == last_data_array[1]) && (rx_array[1] == last_data_array[0]));
}

uint16_t LTC2664_voltage_to_code(float dac_voltage, float min_output, float max_output)
// Calculate a LTC2664 DAC code given the desired output voltage and the minimum / maximum
// outputs for a given softspan range.
{
  uint16_t dac_code;
  float float_code;
  float_code = 4095.0 * (dac_voltage - min_output) / (max_output - min_output);                    // Calculate the DAC code
  float_code = (float_code > (floor(float_code) + 0.5)) ? ceil(float_code) : floor(float_code);     // Round
  if (float_code < 0.0) float_code = 0.0;
  if (float_code > 4095.0) float_code = 4095.0; 
  dac_code = (uint16_t) (float_code);                                                             // Convert to unsigned integer
  return (dac_code);
}

float LTC2664_code_to_voltage(uint16_t dac_code, float min_output, float max_output)
// Calculate the LTC2664 DAC output voltage given the DAC code and and the minimum / maximum
// outputs for a given softspan range.
{
  float dac_voltage;
  dac_voltage = (((float) dac_code / 4095.0) * (max_output - min_output)) + min_output;            // Calculate the dac_voltage
  return (dac_voltage);
}

void set_opa(opa_gain gain)
{
    GPIO_PinState a2 = GPIO_PIN_RESET;
    GPIO_PinState a1 = GPIO_PIN_RESET;
    GPIO_PinState a0 = GPIO_PIN_RESET;

    switch(gain) {
        case OPA_SHUTDOWN:   // A2 A1 A0 = 1 1 1
            a2 = GPIO_PIN_SET;
            a1 = GPIO_PIN_SET;
            a0 = GPIO_PIN_SET;
            break;
        case OPA_GAIN_025:   // 0.25 V/V: A2 A1 A0 = 1 1 0
            a2 = GPIO_PIN_SET;
            a1 = GPIO_PIN_SET;
            a0 = GPIO_PIN_RESET;
            break;
        case OPA_GAIN_05:    // 0.5 V/V:  A2 A1 A0 = 1 0 1
            a2 = GPIO_PIN_SET;
            a1 = GPIO_PIN_RESET;
            a0 = GPIO_PIN_SET;
            break;
        case OPA_GAIN_1:     // 1 V/V:    A2 A1 A0 = 1 0 0
            a2 = GPIO_PIN_SET;
            a1 = GPIO_PIN_RESET;
            a0 = GPIO_PIN_RESET;
            break;
        case OPA_GAIN_2:     // 2 V/V:    A2 A1 A0 = 0 1 1
            a2 = GPIO_PIN_RESET;
            a1 = GPIO_PIN_SET;
            a0 = GPIO_PIN_SET;
            break;
        case OPA_GAIN_4:     // 4 V/V:    A2 A1 A0 = 0 1 0
            a2 = GPIO_PIN_RESET;
            a1 = GPIO_PIN_SET;
            a0 = GPIO_PIN_RESET;
            break;
        case OPA_GAIN_8:     // 8 V/V:    A2 A1 A0 = 0 0 1
            a2 = GPIO_PIN_RESET;
            a1 = GPIO_PIN_RESET;
            a0 = GPIO_PIN_SET;
            break;
        case OPA_GAIN_16:    // 16 V/V:   A2 A1 A0 = 0 0 0
        default:
            a2 = GPIO_PIN_RESET;
            a1 = GPIO_PIN_RESET;
            a0 = GPIO_PIN_RESET;
            break;
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, a0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, a1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  a2);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, a0);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, a1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, a2);
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, a0);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, a1);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, a2);
}
  
