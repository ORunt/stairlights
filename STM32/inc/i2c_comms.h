/**
  ******************************************************************************
  * @file    i2c_comms.h
  * @author  MCD Application Team and Cam Sharp mods
  * @version V1.1.1
  * @date    16-January-2014
  * @brief   This file contains all the functions prototypes for the
  *          i2c_comms.c firmware driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_COMMS_H
#define __I2C_COMMS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/* Exported constants --------------------------------------------------------*/
/* Uncomment the following line to use Timeout_User_Callback I2C_TimeoutUserCallback().
   If This Callback is enabled, it should be implemented by user in main function .
   I2C_TimeoutUserCallback() function is called whenever a timeout condition
   occurs during communication (waiting on an event that doesn't occur, bus
   errors, busy devices ...). */
/* #define USE_TIMEOUT_USER_CALLBACK */

/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define I2C_LONG_TIMEOUT         ((uint32_t)(10 * I2C_FLAG_TIMEOUT))


/**
  * @brief  Block Size
  */
#define I2C_TIMEOUT         ((uint32_t)0x3FFFF) /* I2C Time out */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void HL_I2C_DeInit(uint8_t sda_pin, uint8_t scl_pin);
uint8_t HL_I2C_GetStatus(uint16_t addr);
uint8_t HL_I2C_Receive(uint16_t addr, uint8_t regName, uint8_t * data, uint32_t count);
uint8_t HL_I2C_Transmit(uint16_t addr, uint8_t regName, uint8_t * data, uint32_t count);

/**
  * @brief  Timeout user callback function. This function is called when a timeout
  *         condition occurs during communication with the I2C sensor. Only protoype
  *         of this function is decalred in the I2C sensor driver. Its implementation
  *         may be done into user application. This function may typically stop
  *         current operations and reset the I2C peripheral and sensor.
  *         To enable this function use uncomment the define USE_TIMEOUT_USER_CALLBACK
  *         at the top of this file.
  */
#ifdef USE_TIMEOUT_USER_CALLBACK
 uint8_t I2C_TIMEOUT_UserCallback(void);
#else
 #define I2C_TIMEOUT_UserCallback()  1
#endif /* USE_TIMEOUT_USER_CALLBACK */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_COMMS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
