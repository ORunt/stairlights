/**
  ******************************************************************************
  * @file    i2c_comms.c
  * @author  MCD Application Team and Cam Sharp mods
  * @version V1.1.1
  * @date    16-January-2014
  * @brief   This file provides a set of functions needed to manage the I2C LM75
  *          temperature sensor mounted on STM320518-EVAL board.
  *          It implements a high level communication layer for read and write
  *          from/to this sensor. The needed STM32F0xx hardware resources (I2C and
  *          GPIO) are defined in stm320518_eval.h file, and the initialization is
  *          performed in LM75_LowLevel_Init() function declared in stm320518_eval.c
  *          file.
  *          You can easily tailor this driver to any other development board,
  *          by just adapting the defines for hardware resources and
  *          LM75_LowLevel_Init() function.
  *
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

/* Includes ------------------------------------------------------------------*/
#include "i2c_comms.h"


__IO uint32_t I2CTimeout = I2C_LONG_TIMEOUT;

/**
  * @brief  DeInitializes the I2C1.
  * @param  None
  * @retval None
  */
void HL_I2C_DeInit(uint8_t sda_pin, uint8_t scl_pin)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Disable I2C1 */
    I2C_Cmd(I2C1, DISABLE);

    /* DeInitializes the I2C1 */
    I2C_DeInit(I2C1);

    /* I2C1 Periph clock disable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);

    /* Configure I2C1 pins: SCL */
    GPIO_InitStructure.GPIO_Pin = 1<<scl_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure I2C1 pins: SDA */
    GPIO_InitStructure.GPIO_Pin = 1<<sda_pin;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/**
  * @brief  Checks the status of the I2C sensor.
  * @param  None
  * @retval ErrorStatus: LM75 Status (ERROR or SUCCESS).
  */
uint8_t HL_I2C_GetStatus(uint16_t addr)
{
  uint32_t I2C_TimeOut = I2C_TIMEOUT;

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C1, addr, 0, I2C_AutoEnd_Mode, I2C_No_StartStop);

  /* Clear NACKF and STOPF */
  I2C_ClearFlag(I2C1, I2C_ICR_NACKCF | I2C_ICR_STOPCF);

  /* Generate start */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Wait until timeout elapsed */
  while ((I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET) && (I2C_TimeOut-- != 0));

  /* Check if I2C sensor is ready for use */
  if ((I2C_GetFlagStatus(I2C1, I2C_ISR_NACKF) != RESET) || (I2C_TimeOut == 0))
  {
    /* Clear NACKF and STOPF */
    I2C_ClearFlag(I2C1, I2C_ICR_NACKCF | I2C_ICR_STOPCF);

    return 1;
  }
  else
  {
    /* Clear STOPF */
    I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

    return 0;
  }
}

/**
  * @brief  Read the specified register from the I2C device.
  * @param  addr: device address.
  * @param  regName: specifies the register to be read.
  * @param  data: The data buffer to read into.
  * @param  count: Number of bytes to read
  * @retval error code.
  */
uint8_t HL_I2C_Receive(uint16_t addr, uint8_t regName, uint8_t * data, uint32_t count)
{
  uint32_t DataNum = 0;

  /* Test on BUSY Flag */
  I2CTimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET)
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C1, addr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  I2CTimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET)
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Send Register address */
  I2C_SendData(I2C1, (uint8_t)regName);

  /* Wait until TC flag is set */
  I2CTimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET)
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C1, addr, count, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Reset local variable */
  DataNum = 0;

  /* Wait until all data are received */
  while (DataNum != count)
  {
    /* Wait until RXNE flag is set */
    I2CTimeout = I2C_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET)
    {
      if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
    }

    /* Read data from RXDR */
    data[DataNum]= I2C_ReceiveData(I2C1);

    /* Update number of received data */
    DataNum++;
  }

  /* Wait until STOPF flag is set */
  I2CTimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET)
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

  /* return a error code */
  return 0;
}

/**
  * @brief  Write to the specified register of the LM75.
  * @param  addr: device address.
  * @param  regName: specifies the register to be written to.
  * @param  data: The data buffer to write from.
  * @param  count: Number of bytes to write
  * @retval None
  */
uint8_t HL_I2C_Transmit(uint16_t addr, uint8_t regName, uint8_t * data, uint32_t count)
{
  uint32_t DataNum = 0;

  /* Test on BUSY Flag */
  I2CTimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET)
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C1, addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  I2CTimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET)
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Send Register address */
  I2C_SendData(I2C1, (uint8_t)regName);

  /* Wait until TCR flag is set */
  I2CTimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TCR) == RESET)
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C1, addr, count, I2C_AutoEnd_Mode, I2C_No_StartStop);

  while (DataNum != count)
  {
    /* Wait until TXIS flag is set */
    I2CTimeout = I2C_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET)
    {
      if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
    }

    /* Write data to TXDR */
    I2C_SendData(I2C1, (uint8_t)(data[DataNum]));

    /* Update number of transmitted data */
    DataNum++;
  }

  /* Wait until STOPF flag is set */
  I2CTimeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET)
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

  return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
