#include "func_vl53l0x.h"
#include "func_debug.h"
#include "vl53l0x.h"
#include "i2c_comms.h"

#ifdef TRIGGER_VL53L0X

//#define TRIGGER_VL53L0X_INT     // Define to use the TOF sensors to trigger stair lights in Interrupt mode
                                // Undefine for polling mode
#define ENABLE_VL53L0X_TOP      // Enable sensor 2
#define ENABLE_VL53L0X_BOTTOM   // Enable sensor 1



vl53l0x_data vl53l0x_bottom;
vl53l0x_data vl53l0x_top;
volatile bool tof_trigger_top = 0;
volatile bool tof_trigger_bottom = 0;

// THRESHOLD_LOW (mm), THRESHOLD_HIGH (mm), THRESHOLD_FILTER (mm)
const vl53l0x_threshold_t tof_thresh_top =      {400, 1500, 300};
const vl53l0x_threshold_t tof_thresh_bottom =   {600, 1500, 200};

  /**
  * @brief  Initializes the I2C on Port B.
  * @param  sda_pin i2c data pin number (decimal)
  * @param  scl_pin i2c clk pin number (decimal)
  * @retval None
  */
static void I2C_Config(void)
{
    // =========== Setup I2C Interrupts ===========
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Reconfigure and enable I2C1 error interrupt to have the higher priority */
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // =========== Setup GPIO's for I2C ===========
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* I2C1 Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* Configure the I2C clock source. The clock is derived from the HSI */
    RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

    /* I2C_SCL_GPIO_CLK, I2C_SDA_GPIO_CLK Periph clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Connect PXx to SCL */
    GPIO_PinAFConfig(GPIOB, GPIO_I2C_SCL, GPIO_AF_1);

    /* Connect PXx to SDA */
    GPIO_PinAFConfig(GPIOB, GPIO_I2C_SDA, GPIO_AF_1);

    /* Configure I2C pins: SCL */
    GPIO_InitStructure.GPIO_Pin = 1<<GPIO_I2C_SCL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure I2C pins: SDA */
    GPIO_InitStructure.GPIO_Pin = 1<<GPIO_I2C_SDA;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Setup X-SHUT pins
    GPIO_InitStructure.GPIO_Pin = GPIO_XSHUT_TOP_PIN | GPIO_XSHUT_BOTTOM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	  	// input
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// pushpull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// max
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	    // Pullup resistors
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIOB->BRR = GPIO_XSHUT_TOP_PIN | GPIO_XSHUT_BOTTOM_PIN;    // Set Low

    // =========== Setup I2C Config ===========
    I2C_InitTypeDef  I2C_InitStructure;

    /* I2C1 configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    //I2C_InitStructure.I2C_Timing = 0x0010020A;    // 344khz
    //I2C_InitStructure.I2C_Timing = 0xF000F3FF;    // 1khz
    I2C_InitStructure.I2C_Timing = 0x00201D7B;      // 50kHz - most stable so far
    //I2C_InitStructure.I2C_Timing = 0x00201D2B;    // 100khz
    //I2C_InitStructure.I2C_Timing = 0x0010021E;    // 200khz

    /* Apply I2C1 configuration after enabling it */
    I2C_Init(I2C1, &I2C_InitStructure);

    /* I2C1 Peripheral Enable */
    I2C_Cmd(I2C1, ENABLE);

    /* I2C1 Interrupts enable for errors - do we actually need this? */
    I2C_ITConfig(I2C1, I2C_IT_ERRI, ENABLE);

}

void I2C1_IRQHandler(void)
{
    uint8_t it_cleared = 1;
    /* Check on I2C1 Time out flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT))
    {
        it_cleared = 0;
        I2C_ClearITPendingBit(I2C1, I2C_IT_TIMEOUT);
    }
    /* Check on I2C1 Arbitration Lost flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_ARLO))
    {
        it_cleared = 0;
        I2C_ClearITPendingBit(I2C1, I2C_IT_ARLO);
    }
    /* Check on I2C1 PEC error flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_PECERR))
    {
        it_cleared = 0;
        I2C_ClearITPendingBit(I2C1, I2C_IT_PECERR);
    }
    /* Check on I2C1 Overrun/Underrun error flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_OVR))
    {
        it_cleared = 0;
        I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
    }
    /* Check on I2C1 Acknowledge failure error flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_NACKF))
    {
        it_cleared = 0;
        I2C_ClearITPendingBit(I2C1, I2C_IT_NACKF);
    }
    /* Check on I2C1 Bus error flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_BERR))
    {
        it_cleared = 0;
        I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
    }
    DBG_ERR_CHK(it_cleared, 5);
    DBG_ERR_CHK(!it_cleared, 6);
}

#ifdef TRIGGER_VL53L0X_INT
static void EXTI0_Config(void)
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    /* Enable GPIOA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Configure PA0 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = (1<<GPIO_TOF_INT_TOP_PIN) | (1<<GPIO_TOF_INT_BOTTOM_PIN);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      // Active high
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    /* Connect EXTI0 Line to PB1 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_TOF_INT_TOP_PIN);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_TOF_INT_BOTTOM_PIN);

    /* Configure EXTI0 line */
    EXTI_InitStructure.EXTI_Line = (1<<GPIO_TOF_INT_TOP_PIN);
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = (1<<GPIO_TOF_INT_BOTTOM_PIN);
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_Disable(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;      // Disable the IRQ channel
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;    // Disable I2Cx EVT IRQn
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  This function handles External line 0 to 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_1_IRQHandler(void)
{
    if(EXTI_GetITStatus(1<<GPIO_TOF_INT_TOP_PIN) != RESET)
    {
        tof_trigger_top = 1;

        // Clear the EXTI line 1 pending bit
        EXTI_ClearITPendingBit(1<<GPIO_TOF_INT_TOP_PIN);

        VL53L0X_ClearInterruptMask(&vl53l0x_top.vl53l0x, -1);
    }
    if(EXTI_GetITStatus(1<<GPIO_TOF_INT_BOTTOM_PIN) != RESET)
    {
        tof_trigger_bottom = 1;

        // Clear the EXTI line 2 pending bit
        EXTI_ClearITPendingBit(1<<GPIO_TOF_INT_BOTTOM_PIN);

        VL53L0X_ClearInterruptMask(&vl53l0x_bottom.vl53l0x, -1);
    }
}

void startTof(bool start)
{
    if (start)
    {
#ifdef ENABLE_VL53L0X_BOTTOM
        vl53l0x_start_continuous_interrupt_measure(&vl53l0x_bottom);
        VL53L0X_ClearInterruptMask(&vl53l0x_bottom.vl53l0x, -1);
#endif
#ifdef ENABLE_VL53L0X_TOP
        vl53l0x_start_continuous_interrupt_measure(&vl53l0x_top);
        VL53L0X_ClearInterruptMask(&vl53l0x_top.vl53l0x, -1);
#endif
    }
    else
    {
#ifdef ENABLE_VL53L0X_BOTTOM
        vl53l0x_stop_continuous_interrupt_measure(&vl53l0x_bottom);
#endif
#ifdef ENABLE_VL53L0X_TOP
        vl53l0x_stop_continuous_interrupt_measure(&vl53l0x_top);
#endif
    }
}

void vl53l0xSetup(void)
{
    I2C_Config();
    EXTI0_Config();
#ifdef ENABLE_VL53L0X_BOTTOM
    vl53l0x_init(&vl53l0x_bottom, 0x30, GPIO_XSHUT_BOTTOM_PIN, LONG_RANGE, MODE_CONT_INTERRUPT, tof_thresh_bottom);
#endif
#ifdef ENABLE_VL53L0X_TOP
    vl53l0x_init(&vl53l0x_top, 0x31, GPIO_XSHUT_TOP_PIN, LONG_RANGE, MODE_CONT_INTERRUPT, tof_thresh_top);
#endif
    startTof(true);
}

int checkToF(direction_e up_down)
{
    uint16_t value = 0;
#ifdef ENABLE_VL53L0X_BOTTOM
    if(up_down == DIR_UP){
        if(tof_trigger_bottom){
            value = 1;
            tof_trigger_bottom = 0;
        }
    }
#endif
#ifdef ENABLE_VL53L0X_TOP
    if(up_down == DIR_DOWN){
        if(tof_trigger_top){
            value = 1;
            tof_trigger_top = 0;
        }
    }
#endif
    return (int)value;
}



#else // Polling mode

void startTof(bool start){}

void vl53l0xSetup(void)
{
    I2C_Config();
#ifdef ENABLE_VL53L0X_BOTTOM
    DBG_ERR_CHK(vl53l0x_init(&vl53l0x_bottom, 0x30, GPIO_XSHUT_BOTTOM_PIN, LONG_RANGE, MODE_SINGLE_SHOT, tof_thresh_bottom), 1);
#endif
#ifdef ENABLE_VL53L0X_TOP
    DBG_ERR_CHK(vl53l0x_init(&vl53l0x_top, 0x31, GPIO_XSHUT_TOP_PIN, LONG_RANGE, MODE_SINGLE_SHOT, tof_thresh_top),2);
#endif
}


int checkToF(direction_e up_down)
{
    uint16_t value = 0;
#ifdef ENABLE_VL53L0X_BOTTOM
    if(up_down == DIR_UP){
        DBG_ERR_CHK(vl53l0x_get_measurement(&vl53l0x_bottom, SENSOR_CHAN_PROX, &value),3);
    }
#endif
#ifdef ENABLE_VL53L0X_TOP
    if(up_down == DIR_DOWN){
        DBG_ERR_CHK(vl53l0x_get_measurement(&vl53l0x_top, SENSOR_CHAN_PROX, &value),4);
    }
#endif
    return value;
}

#endif


#else

void startTof(bool start){}
void vl53l0xSetup(void){}
int checkToF(direction_e up_down){return 0;}

#endif