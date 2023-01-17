#include "main.h"

/*
uint16_t up_arr[100] = {0};
uint16_t down_arr[100] = {0};
int arr_cnt = 0;

int testPeripherals(void)
{
    int result = 0;
#ifdef TRIGGER_BUTTON
    result = checkButtonPress(DIR_UP);
#endif
#ifdef TRIGGER_VL53L0X_POLL
    //result = checkToF(DIR_UP);
    if(arr_cnt<=100)
    {
        //vl53l0x_get_measurement(&vl53l0x_bottom, SENSOR_CHAN_DISTANCE, &up_arr[arr_cnt]);
        vl53l0x_get_measurement(&vl53l0x_top, SENSOR_CHAN_DISTANCE, &down_arr[arr_cnt++]);
    }
    else
        arr_cnt = 5000;
#endif
#ifdef VARIABLE_BRIGHTNESS
    //result = getAdcConversion();
    result = getTimeOfDay();
#endif
    return result;
}*/