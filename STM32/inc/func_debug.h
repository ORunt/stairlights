#ifndef __FUNC_DEBUG_H
#define __FUNC_DEBUG_H

#include "main.h"

#ifdef ENABLE_DEBUG_PRINT
    #include <stdio.h>
    #define DEBUG_PRINT(message, ...)    printf(message, ##__VA_ARGS__); while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}
#else
    #define DEBUG_PRINT(message, ...)
#endif // ENABLE_DEBUG_PRINT

#ifdef ENABLE_DEBUG_LED
    extern const led_channel_t led_channels[LED_CHANNELS];

    #define DBG_ERR_CHK(func,led)       if((func) && ((led)<LED_CHANNELS)){GPIO_SetBits(led_channels[led].port, led_channels[led].pin);DBG_SetErrState(true);}
    #define DBG_ERR_SET(set,led)        DBG_SetErrState(true);if((set) && ((led)<LED_CHANNELS)){GPIO_SetBits(led_channels[led].port, led_channels[led].pin);}else{GPIO_ResetBits(led_channels[led].port, led_channels[led].pin);}
#else
    #define DBG_ERR_CHK(func,led)       func;
    #define DBG_ERR_SET(set,led)
#endif // ENABLE_DEBUG_LED

void printfSetup(void);
bool DBG_GetErrState(void);
void DBG_SetErrState(bool state);

#endif // __FUNC_DEBUG_H