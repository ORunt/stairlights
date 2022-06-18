#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

typedef enum{
    SENSOR_CHAN_ALL,
    SENSOR_CHAN_DISTANCE,
    SENSOR_CHAN_PROX
}sensor_channel;

typedef struct{
	VL53L0X_Dev_t vl53l0x;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
}vl53l0x_data;

int vl53l0x_init(vl53l0x_data *drv_data);
int vl53l0x_get_measurement(vl53l0x_data *drv_data, sensor_channel chan, uint16_t *val);