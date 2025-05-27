#include "debug.h"

extern UART_HandleTypeDef huart8;

int fputc(int ch, FILE *f)
{
 
  HAL_UART_Transmit(&huart8, (uint8_t *)&ch, 1, 0xffff);
 
  return ch;
 
}
