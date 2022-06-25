#ifndef __FUNC_VL53L0X_H
#define __FUNC_VL53L0X_H

#include "main.h"

void startTof(bool start);
void vl53l0xSetup(void);
int checkToF(direction_e up_down);

#endif // __FUNC_VL53L0X_H