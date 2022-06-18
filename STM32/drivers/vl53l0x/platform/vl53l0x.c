#include "stm32f0xx_i2c.h"
#include "vl53l0x.h"

/* All the values used in this driver are coming from ST datasheet and examples.
 * It can be found here:
 *   http://www.st.com/en/embedded-software/stsw-img005.html
 * There are also examples of use in the L4 cube FW:
 *   http://www.st.com/en/embedded-software/stm32cubel4.html
 */
#define VL53L0X_INITIAL_ADDR                    0x29<<1
#define VL53L0X_REG_WHO_AM_I                    0xC0
#define VL53L0X_CHIP_ID                         0xEEAA
#define VL53L0X_SETUP_SIGNAL_LIMIT              (0.1*65536)
#define VL53L0X_SETUP_SIGMA_LIMIT               (60*65536)
#define VL53L0X_SETUP_MAX_TIME_FOR_RANGING      33000
#define VL53L0X_SETUP_PRE_RANGE_VCSEL_PERIOD    18
#define VL53L0X_SETUP_FINAL_RANGE_VCSEL_PERIOD  14

#define CONFIG_VL53L0X_PROXIMITY_THRESHOLD      100 //(mm)

#define ERR_CHK(x)          {uint16_t call_return = (x) ; if (call_return != 0) return call_return;}
#define LOG_DBG(msg, ...)


int vl53l0x_get_measurement(vl53l0x_data *drv_data, sensor_channel chan, uint16_t *val)
{
    ERR_CHK(VL53L0X_PerformSingleRangingMeasurement(&drv_data->vl53l0x, &drv_data->RangingMeasurementData));

	if (chan == SENSOR_CHAN_PROX){
        *val = (drv_data->RangingMeasurementData.RangeMilliMeter <= CONFIG_VL53L0X_PROXIMITY_THRESHOLD);
	}
    else{
		*val = drv_data->RangingMeasurementData.RangeMilliMeter;
	}

	return 0;
}


static int vl53l0x_setup_single_shot(vl53l0x_data *drv_data)
{
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;

	ERR_CHK(VL53L0X_StaticInit(&drv_data->vl53l0x));
	ERR_CHK(VL53L0X_PerformRefCalibration(&drv_data->vl53l0x, &VhvSettings, &PhaseCal));
	ERR_CHK(VL53L0X_PerformRefSpadManagement(&drv_data->vl53l0x, (uint32_t *)&refSpadCount, &isApertureSpads));
	ERR_CHK(VL53L0X_SetDeviceMode(&drv_data->vl53l0x, VL53L0X_DEVICEMODE_SINGLE_RANGING));
	ERR_CHK(VL53L0X_SetLimitCheckEnable(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1));
	ERR_CHK(VL53L0X_SetLimitCheckEnable(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1));
	ERR_CHK(VL53L0X_SetLimitCheckValue(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, VL53L0X_SETUP_SIGNAL_LIMIT));
	ERR_CHK(VL53L0X_SetLimitCheckValue(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, VL53L0X_SETUP_SIGMA_LIMIT));
	ERR_CHK(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&drv_data->vl53l0x, VL53L0X_SETUP_MAX_TIME_FOR_RANGING));
	ERR_CHK(VL53L0X_SetVcselPulsePeriod(&drv_data->vl53l0x, VL53L0X_VCSEL_PERIOD_PRE_RANGE, VL53L0X_SETUP_PRE_RANGE_VCSEL_PERIOD));
	ERR_CHK(VL53L0X_SetVcselPulsePeriod(&drv_data->vl53l0x, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VL53L0X_SETUP_FINAL_RANGE_VCSEL_PERIOD));
}


int vl53l0x_init(vl53l0x_data *drv_data)
{
	VL53L0X_Error ret;
	uint16_t vl53l0x_id = 0U;
	VL53L0X_DeviceInfo_t vl53l0x_dev_info;

    // TODO: Possibly need to setup xshut Device address stuff here

	drv_data->vl53l0x.I2cDevAddr = VL53L0X_INITIAL_ADDR;

    if(VL53L0X_GetStatus(drv_data))
    {
        return 1;
    }

	/* Get info from sensor */
	(void)memset(&vl53l0x_dev_info, 0, sizeof(VL53L0X_DeviceInfo_t));

	ERR_CHK(VL53L0X_GetDeviceInfo(&drv_data->vl53l0x, &vl53l0x_dev_info));

	LOG_DBG("VL53L0X_GetDeviceInfo = %d", ret);
	LOG_DBG("   Device Name : %s", vl53l0x_dev_info.Name);
	LOG_DBG("   Device Type : %s", vl53l0x_dev_info.Type);
	LOG_DBG("   Device ID : %s", vl53l0x_dev_info.ProductId);
	LOG_DBG("   ProductRevisionMajor : %d", vl53l0x_dev_info.ProductRevisionMajor);
	LOG_DBG("   ProductRevisionMinor : %d", vl53l0x_dev_info.ProductRevisionMinor);

	ret = VL53L0X_RdWord(&drv_data->vl53l0x, VL53L0X_REG_WHO_AM_I, (uint16_t *) &vl53l0x_id);
	if ((ret < 0) || (vl53l0x_id != VL53L0X_CHIP_ID)) {
		return 1;
	}

	/* sensor init */
	ERR_CHK(VL53L0X_DataInit(&drv_data->vl53l0x));
	ERR_CHK(vl53l0x_setup_single_shot(drv_data));

	return 0;
}

