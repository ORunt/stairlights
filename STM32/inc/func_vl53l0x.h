#ifndef __FUNC_VL53L0X_H
#define __FUNC_VL53L0X_H

#include "main.h"

void startTof(bool start);
void vl53l0xSetup(void);
int checkToF(direction_e up_down, uint16_t * err_state);

#endif // __FUNC_VL53L0X_H