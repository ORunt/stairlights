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
    uint16_t thresh_low;
    uint16_t thresh_high;
    uint16_t thresh_filt;
}vl53l0x_threshold_t;

typedef struct{
    int16_t last_sample;
    uint8_t cnt;
}vl53l0x_filter;

typedef struct{
	VL53L0X_Dev_t vl53l0x;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    vl53l0x_filter filterData;
    vl53l0x_threshold_t threshData;
}vl53l0x_data;

int vl53l0x_init(vl53l0x_data *drv_data, uint8_t new_adrs, uint16_t xshut_pin, vl53l0x_range range, vl53l0x_mode mode, vl53l0x_threshold_t thresh);
int vl53l0x_get_measurement(vl53l0x_data *drv_data, sensor_channel chan, uint16_t *val);
int vl53l0x_start_continuous_interrupt_measure(vl53l0x_data *drv_data);
int vl53l0x_stop_continuous_interrupt_measure(vl53l0x_data *drv_data);