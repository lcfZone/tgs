#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f4xx_hal.h"
#include "stdio.h"

#define ERR_LEVEL                   1
#define WARN_LEVEL                  2
#define DBG_LEVEL                   3
#define OUT_LEVEL                   4

#define DBG 0
//#define NOP_ACTION        do{}while(0)

#   if !(DBG < ERR_LEVEL)
#       define PRINT_ERR(fmt,arg...)  do{printf(fmt, ##arg); DBG_Send();}while(0)
#   else
#       define PRINT_ERR(fmt,arg...)    
#   endif
#   if !(DBG < WARN_LEVEL)
#       define PRINT_WARN(fmt,arg...)  do{printf(fmt, ##arg); DBG_Send();}while(0)
#   else
#       define PRINT_WARN(fmt,arg...)   
#   endif
#   if !(DBG < DBG_LEVEL)
#       define PRINT_DBG(fmt,arg...)   do{printf(fmt, ##arg); DBG_Send();}while(0)
#   else
#       define PRINT_DBG(fmt,arg...)  
#   endif
#   if !(DBG < OUT_LEVEL)
#       define PRINT_OUT(fmt,arg...)   do{printf(fmt, ##arg); DBG_Send();}while(0)
#   else
#       define PRINT_OUT(fmt,arg...)  
#   endif

int fputc(int ch, FILE *f);
#endif

