#include "stm32f0xx.h"
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

#define FILTER_VL53L0X_SAMPLE_CNT               1   // (We need this many samples within the threshold to trigger an event)

#define ERR_CHK(x)          {uint16_t call_return = (x) ; if (call_return != 0) return call_return;}
#define LOG_DBG(msg, ...)


static uint8_t filterGetProxState(vl53l0x_data *drv_data)
{
    uint8_t ret = 0;
    
    if(drv_data->RangingMeasurementData.RangeStatus)
    {
        // Something went pear shaped
        return 0;
    }
    else
    {
        // If there is a big glitch, don't check the sample
        if(abs((int)drv_data->RangingMeasurementData.RangeMilliMeter - (int)drv_data->filterData.last_sample) > drv_data->threshData.thresh_filt)
        {
            ret = 0;
        }
        else if ((drv_data->RangingMeasurementData.RangeMilliMeter < drv_data->threshData.thresh_high) && 
                 (drv_data->RangingMeasurementData.RangeMilliMeter > drv_data->threshData.thresh_low))
        {
            // Check how many samples we need within the threshold to give a valid state transition
            if(++drv_data->filterData.cnt >= FILTER_VL53L0X_SAMPLE_CNT)
            {
                drv_data->filterData.cnt = 0;
                ret = 1;
            }
        }
        drv_data->filterData.last_sample = drv_data->RangingMeasurementData.RangeMilliMeter;
    }
    
    return ret;
}

int vl53l0x_get_measurement(vl53l0x_data *drv_data, sensor_channel chan, uint16_t *val)
{
    *val = 0;       // Reset
    
    ERR_CHK(VL53L0X_PerformSingleRangingMeasurement(&drv_data->vl53l0x, &drv_data->RangingMeasurementData));
    
    if (drv_data->RangingMeasurementData.RangeMilliMeter == 0)
    {
        return 1;   // Unknown error
    }
    
	if (chan == SENSOR_CHAN_PROX){
        *val = (int)filterGetProxState(drv_data);
	}
    else{
		*val = (int)drv_data->RangingMeasurementData.RangeMilliMeter;
	}

	return 0;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted = 0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}


static int vl53l0x_setup_single_shot(vl53l0x_data *drv_data, vl53l0x_range range)
{
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;

	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

    switch(range)
    {
        case HIGH_ACCURACY:
            signalLimit = (FixPoint1616_t)(0.25*65536);
            sigmaLimit = (FixPoint1616_t)(18*65536);
            timingBudget = 200000;
            preRangeVcselPeriod = 14;
            finalRangeVcselPeriod = 10;
            break;
        case HIGH_SPEED:
            signalLimit = (FixPoint1616_t)(0.25*65536);
            sigmaLimit = (FixPoint1616_t)(32*65536);
            timingBudget = 20000;
            preRangeVcselPeriod = 14;
            finalRangeVcselPeriod = 10;
            break;
        case LONG_RANGE:
        default:
            signalLimit = (FixPoint1616_t)(0.1*65536);
            sigmaLimit = (FixPoint1616_t)(60*65536);
            timingBudget = 33000;
            preRangeVcselPeriod = 18;
            finalRangeVcselPeriod = 14;
            break;
    }

	ERR_CHK(VL53L0X_StaticInit(&drv_data->vl53l0x));
	ERR_CHK(VL53L0X_PerformRefCalibration(&drv_data->vl53l0x, &VhvSettings, &PhaseCal));
	ERR_CHK(VL53L0X_PerformRefSpadManagement(&drv_data->vl53l0x, (uint32_t *)&refSpadCount, &isApertureSpads));
	ERR_CHK(VL53L0X_SetDeviceMode(&drv_data->vl53l0x, VL53L0X_DEVICEMODE_SINGLE_RANGING));
	ERR_CHK(VL53L0X_SetLimitCheckEnable(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1));
	ERR_CHK(VL53L0X_SetLimitCheckEnable(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1));
	ERR_CHK(VL53L0X_SetLimitCheckValue(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit));
	ERR_CHK(VL53L0X_SetLimitCheckValue(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit));
	ERR_CHK(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&drv_data->vl53l0x, timingBudget));
	ERR_CHK(VL53L0X_SetVcselPulsePeriod(&drv_data->vl53l0x, VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod));
	ERR_CHK(VL53L0X_SetVcselPulsePeriod(&drv_data->vl53l0x, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod));
    return 0;
}

static int vl53l0x_setup_continuous_interrupt(vl53l0x_data *drv_data)
{
    uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;

    /* Initialize the device in continuous ranging mode */
	ERR_CHK(VL53L0X_StaticInit(&drv_data->vl53l0x));
	ERR_CHK(VL53L0X_PerformRefCalibration(&drv_data->vl53l0x, &VhvSettings, &PhaseCal));
	ERR_CHK(VL53L0X_PerformRefSpadManagement(&drv_data->vl53l0x, &refSpadCount, &isApertureSpads));
	ERR_CHK(VL53L0X_SetInterMeasurementPeriodMilliSeconds(&drv_data->vl53l0x, 250));
	ERR_CHK(VL53L0X_SetDeviceMode(&drv_data->vl53l0x, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING));
    return 0;
}

int vl53l0x_start_continuous_interrupt_measure(vl53l0x_data *drv_data)
{
    int status = 0;
    /* set sensor interrupt mode */
    VL53L0X_StopMeasurement(&drv_data->vl53l0x);           // it is safer to do this while sensor is stopped
    //                                                                                          // LOW THRESH                       // HIGH THRESH
    VL53L0X_SetInterruptThresholds(&drv_data->vl53l0x, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,  (drv_data->threshData.thresh_high<<16),  (0<<16));
    status = VL53L0X_SetGpioConfig(&drv_data->vl53l0x, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW, VL53L0X_INTERRUPTPOLARITY_HIGH);
    status = VL53L0X_ClearInterruptMask(&drv_data->vl53l0x, -1); // clear interrupt pending if any

    /* Start continuous ranging */
    VL53L0X_StartMeasurement(&drv_data->vl53l0x);

    return status;
}

int vl53l0x_stop_continuous_interrupt_measure(vl53l0x_data *drv_data)
{
    /* Stop continuous ranging */
    VL53L0X_StopMeasurement(&drv_data->vl53l0x);

    /* Ensure device is ready for other commands */
    WaitStopCompleted(&drv_data->vl53l0x);
}

int vl53l0x_init(vl53l0x_data *drv_data, uint8_t new_adrs, uint16_t xshut_pin, vl53l0x_range range, vl53l0x_mode mode, vl53l0x_threshold_t thresh)
{
	VL53L0X_Error ret;
	uint16_t vl53l0x_id = 0U;
	VL53L0X_DeviceInfo_t vl53l0x_dev_info;
    new_adrs = new_adrs<<1;

	drv_data->vl53l0x.I2cDevAddr = VL53L0X_INITIAL_ADDR;
    drv_data->threshData = thresh;

    GPIOB->BSRR = xshut_pin;

    // Set to 400Khz before changing addresses or something?
    //VL53L0X_WrByte(&drv_data->vl53l0x, 0x88, 0x00);

    if(VL53L0X_GetStatus(&drv_data->vl53l0x))
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

    ERR_CHK(VL53L0X_SetDeviceAddress(&drv_data->vl53l0x, new_adrs));
    drv_data->vl53l0x.I2cDevAddr = new_adrs;

    ret = VL53L0X_RdWord(&drv_data->vl53l0x, VL53L0X_REG_WHO_AM_I, (uint16_t *) &vl53l0x_id);
	if ((ret < 0) || (vl53l0x_id != VL53L0X_CHIP_ID)) {
		return 1;
	}

	/* sensor init */
	ERR_CHK(VL53L0X_DataInit(&drv_data->vl53l0x));
    drv_data->vl53l0x.Present = 1;

    switch(mode)
    {
        case MODE_SINGLE_SHOT:
            ERR_CHK(vl53l0x_setup_single_shot(drv_data, range));
            break;
        case MODE_CONT_INTERRUPT:
            ERR_CHK(vl53l0x_setup_continuous_interrupt(drv_data));
            break;
        default:
            return 1;
    }

	return 0;
}

