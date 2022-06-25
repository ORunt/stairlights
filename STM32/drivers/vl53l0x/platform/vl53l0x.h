#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

typedef enum{
    SENSOR_CHAN_ALL,
    SENSOR_CHAN_DISTANCE,
    SENSOR_CHAN_PROX
}sensor_channel;

typedef enum{
    LONG_RANGE,
    HIGH_ACCURACY,
    HIGH_SPEED
}vl53l0x_range;

typedef enum{
    MODE_SINGLE_SHOT,
    MODE_CONT_INTERRUPT
}vl53l0x_mode;

typedef struct{
	VL53L0X_Dev_t vl53l0x;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
}vl53l0x_data;

int vl53l0x_init(vl53l0x_data *drv_data, uint8_t new_adrs, uint16_t xshut_pin, vl53l0x_range range, vl53l0x_mode mode);
int vl53l0x_get_measurement(vl53l0x_data *drv_data, sensor_channel chan, uint16_t *val);
int vl53l0x_start_continuous_interrupt_measure(vl53l0x_data *drv_data);
int vl53l0x_stop_continuous_interrupt_measure(vl53l0x_data *drv_data);