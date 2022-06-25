#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

#include "i2c_comms.h"
#include <string.h>

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1
//#define VL53L0X_OsDelay(...) HAL_Delay(2)


#ifndef HAL_I2C_MODULE_ENABLED
#warning "HAL I2C module must be enable "
#endif

/* when not customized by application define dummy one */
#ifndef VL53L0X_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_GetI2cBus(...) (void)0
#endif

#ifndef VL53L0X_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_PutI2cBus(...) (void)0
#endif

//#ifndef VL53L0X_OsDelay
//#   define  VL53L0X_OsDelay(...) (void)0
//#endif


uint8_t _I2CBuffer[64];

void VL53L0X_OsDelay(void)
{
    // 2000us x 64mHz clock / 4 clock cycles for while loop
    int count = (2000 * 64) / 4;
    int i;
    for(i = 0; i < count; i++) {
        count--;
    }
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (count > sizeof(_I2CBuffer)) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    memcpy(&_I2CBuffer[0], pdata, count);

    if(HL_I2C_Transmit(Dev->I2cDevAddr, index, _I2CBuffer, count)){
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    else{
        return VL53L0X_ERROR_NONE;
    }
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if(HL_I2C_Receive(Dev->I2cDevAddr, index, pdata, count)){
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    else{
        return VL53L0X_ERROR_NONE;
    }
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    _I2CBuffer[0] = data;

    if(HL_I2C_Transmit(Dev->I2cDevAddr, index, _I2CBuffer, 1)){
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    else{
        return VL53L0X_ERROR_NONE;
    }
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    _I2CBuffer[0] = data >> 8;
    _I2CBuffer[1] = data & 0x00FF;

    if(HL_I2C_Transmit(Dev->I2cDevAddr, index, _I2CBuffer, 2)){
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    else{
        return VL53L0X_ERROR_NONE;
    }
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    _I2CBuffer[0] = (data >> 24) & 0xFF;
    _I2CBuffer[1] = (data >> 16) & 0xFF;
    _I2CBuffer[2] = (data >> 8)  & 0xFF;
    _I2CBuffer[3] = (data >> 0 ) & 0xFF;

    if(HL_I2C_Transmit(Dev->I2cDevAddr, index, _I2CBuffer, 4)){
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    else{
        return VL53L0X_ERROR_NONE;
    }
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L0X_WrByte(Dev, index, data);
done:
    return Status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    if(HL_I2C_Receive(Dev->I2cDevAddr, index, data, 1)){
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    else{
        return VL53L0X_ERROR_NONE;
    }
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    if(HL_I2C_Receive(Dev->I2cDevAddr, index, _I2CBuffer, 2)){
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    else{
        *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];
        return VL53L0X_ERROR_NONE;
    }
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    if(HL_I2C_Receive(Dev->I2cDevAddr, index, _I2CBuffer, 4)){
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    else{
        *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];
        return VL53L0X_ERROR_NONE;
    }
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    // do nothing
    VL53L0X_OsDelay();
    return status;
}
VL53L0X_Error VL53L0X_GetStatus(VL53L0X_DEV Dev) {
    return (VL53L0X_Error)HL_I2C_GetStatus(Dev->I2cDevAddr);
}
//end of file
