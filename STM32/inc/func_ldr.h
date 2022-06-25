#ifndef __FUNC_LDR_H
#define __FUNC_LDR_H

#include "main.h"

void ldrSetup(void);
time_e setTimeOfDay(uint32_t * fade_times, uint8_t * max_brightness);

#endif // __FUNC_LDR_H