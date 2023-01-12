#ifndef __FUNC_DEBUG_H
#define __FUNC_DEBUG_H

#include "main.h"

#ifdef ENABLE_DEBUG_PRINT
    #include <stdio.h>
    #define DEBUG_PRINT(message, ...)    printf(message, ##__VA_ARGS__); while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}
#else
    #define DEBUG_PRINT(message, ...)
#endif // ENABLE_DEBUG_PRINT

void printfSetup(void);

#endif // __FUNC_DEBUG_H