#include "stairlight_functions.h"
#include "graph_lookup_table.h"
#include "stm32_i2c_high_level.h"
#include "vl53l0x.h"

// PSC = ceil((64Mhz x 10us / 160) - 1)
// PSC = ceil((8Mhz x 10us / 40) - 1)
#define CONVERT_2_COUNT(ms) ((ms) * (FREQUENCY/10))
#define WHILE_WAIT          CONVERT_2_COUNT(STEPS_DURATION + FADE_DURATION)
#define PULSE_PERIOD        (10000/FREQUENCY)      // The pulse period of the main timer in (us)
#define AC_DIM_PRESCALER	1
#define ARR                 (PULSE_PERIOD * 16)

// GPIO defines
#define GPIO_PUSH_BUTTONS_IN    GPIO_Pin_1 | GPIO_Pin_2 //GPIO_Pin_8 | GPIO_Pin_9
#define GPIO_PUSH_BUTTONS_OUT   GPIO_Pin_3 | GPIO_Pin_4 //GPIO_Pin_10 | GPIO_Pin_11
#define GPIO_I2C_SCL            GPIO_PinSource8
#define GPIO_I2C_SDA            GPIO_PinSource9
#define GPIO_ADC_IN             GPIO_Pin_0
#define ADC_DAY_TIME_THRESHOLD  500 // Max is 1023

typedef struct{
    uint8_t offset;
    uint8_t percent;
}led_brightness_t;

const led_channel_t led_channels[LED_CHANNELS] = {  {GPIOA, GPIO_Pin_0},
                                                    {GPIOA, GPIO_Pin_1},
                                                    {GPIOA, GPIO_Pin_2},
                                                    {GPIOA, GPIO_Pin_3},
                                                    {GPIOA, GPIO_Pin_4},
                                                    {GPIOA, GPIO_Pin_5},
                                                    {GPIOA, GPIO_Pin_6},
                                                    {GPIOA, GPIO_Pin_7},
                                                    {GPIOA, GPIO_Pin_8},
                                                    {GPIOA, GPIO_Pin_9},
                                                    {GPIOB, GPIO_Pin_6},
                                                    {GPIOB, GPIO_Pin_7}};

volatile uint8_t led_brightness[LED_CHANNELS] = {0};
uint8_t tim3_counter = 0;  // increments in 10us steps
volatile uint32_t us_timer_counter = 0;
vl53l0x_data vl53l0x_bottom;
vl53l0x_data vl53l0x_top;
uint32_t * fade_times = &fade_times_day[0];
uint8_t max_brightness = MAX_BRIGHTNESS_DAY;

static void CLK_Config()
{
    // Set up 64 MHz Core Clock using HSI (8Mhz) with PLL x 8
    RCC_PLLConfig(RCC_PLLSource_HSI, RCC_PLLMul_8);
    RCC_PLLCmd(ENABLE);

    // Wait for PLLRDY after enabling PLL.
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET)
    { }

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  // Select the PLL as clock source.
    SystemCoreClockUpdate();
}

static void TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = ARR;                  // ARR
    TIM_TimeBaseStructure.TIM_Prescaler = AC_DIM_PRESCALER; // PSC
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* Output Compare Timing Mode configuration: Channel1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;
    TIM_OCInitStructure.TIM_Pulse = ARR;                      // Output compare value (CCR1)
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* TIM Interrupts enable */
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);
}

static void GPIO_ButtonSetup(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    /* Configure PB8 - PB9 in input mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_PUSH_BUTTONS_IN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	  	// input
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// pushpull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// max
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	    // Pulldown resistors
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure PB10 - PB11 as output ground */
    GPIO_InitStructure.GPIO_Pin = GPIO_PUSH_BUTTONS_OUT;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	  	// output
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // output should not have pull up/down
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIOB->BSRR = GPIO_PUSH_BUTTONS_OUT;                // Set High
}

static void ADC_Config(void)
{
    ADC_InitTypeDef     ADC_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;

    /* GPIOB Periph clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* ADC1 Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_ADC_IN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ADCs DeInit */
    ADC_DeInit(ADC1);

    /* Initialize ADC structure */
    ADC_StructInit(&ADC_InitStructure);

    /* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Convert the ADC1 Channel 8 with 239.5 Cycles as sampling time */
    ADC_ChannelConfig(ADC1, ADC_Channel_8 , ADC_SampleTime_239_5Cycles);

    /* ADC Calibration */
    ADC_GetCalibrationFactor(ADC1);

    /* Enable the ADC peripheral */
    ADC_Cmd(ADC1, ENABLE);

    /* Wait the ADRDY flag */
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

    /* ADC1 regular Software Start Conv */
    ADC_StartOfConversion(ADC1);
}


static time_e getAdcConversion(void)
{
    ADC_StartOfConversion(ADC1);

    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    /* Get ADC1 converted data */
    return (ADC_GetConversionValue(ADC1) > ADC_DAY_TIME_THRESHOLD) ? TIME_DAY : TIME_NIGHT;
}

static void GPIO_Config(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    /* Enable GPIOA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Configure PA1 - PA9 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |\
                                  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	  	// output
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// pushpull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// max
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	// output should not have pull up/down
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure PB6 - PB7 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}

static void I2C_Config(void)
{
    HL_I2C_Init(GPIO_I2C_SDA, GPIO_I2C_SCL);
}

void stairlightSetup(void)
{
    CLK_Config();
    GPIO_Config();
#ifdef TRIGGER_VL53L0X
    I2C_Config();
    vl53l0x_init(&vl53l0x_bottom);
#endif
#ifdef TRIGGER_BUTTON
    GPIO_ButtonSetup();
#endif
    ADC_Config();
    TIM_Config();
}

//#define CHECK_FREQ

void TIM3_IRQHandler(void)
{
    // Output compare 1
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        int i;

        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        #ifndef CHECK_FREQ
        for(i = 0; i < LED_CHANNELS; i++)
        {
            if(tim3_counter == 0)
            {
                if(led_brightness[i] == 0)
                {
                    GPIO_ResetBits(led_channels[i].port, led_channels[i].pin);
                }
                else
                {
                    GPIO_SetBits(led_channels[i].port, led_channels[i].pin);
                }
            }
            else if(led_brightness[i] == tim3_counter)
            {
                GPIO_ResetBits(led_channels[i].port, led_channels[i].pin);
            }
        }
        #else
        static uint8_t cam = 0;
        if(cam == 1)
            GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        else
            GPIO_SetBits(GPIOA, GPIO_Pin_0);
        cam ^= 1;
        #endif

        tim3_counter = (tim3_counter + 1) % 100;
        us_timer_counter++;
    }
}

void I2C1_IRQHandler(void)
{
    /* Check on I2C1 Time out flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_TIMEOUT);
    }
    /* Check on I2C1 Arbitration Lost flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_ARLO))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_ARLO);
    }
    /* Check on I2C1 PEC error flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_PECERR))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_PECERR);
    }
    /* Check on I2C1 Overrun/Underrun error flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_OVR))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
    }
    /* Check on I2C1 Acknowledge failure error flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_NACKF))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_NACKF);
    }
    /* Check on I2C1 Bus error flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_BERR))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
    }
}

static uint32_t setTimeOfDay(void)
{
    time_e time_of_day = getAdcConversion();
    switch(time_of_day)
    {
        case TIME_DAY:
            fade_times = &fade_times_day[0];
            max_brightness = MAX_BRIGHTNESS_DAY;
        break;
        case TIME_NIGHT:
            fade_times = &fade_times_night[0];
            max_brightness = MAX_BRIGHTNESS_NIGHT;
        break;
        default: break;
    }

    return time_of_day;
}

void setLedBrightness(uint8_t channel, uint8_t brightness)
{
    if(channel >= LED_CHANNELS)
    {
        return;
    }
    if(brightness > 100)
    {
        return;
    }
    led_brightness[channel] = brightness;
}


static void startOnFadeUp(void)
{
    int i;
    uint32_t temp_counter = 0;
    us_timer_counter = 0;

    while (temp_counter <= WHILE_WAIT)
    {
        for(i = 0; i < LED_CHANNELS; i++)
        {
            if(led_brightness[i] < max_brightness)
            {
                // TODO Do i need to add some delta +- times here encase the interrupts are too faaast?
                if(temp_counter >= (fade_times[led_brightness[i]] + stair_times[i]))
                {
                    led_brightness[i]++;
                }
            }
        }
        temp_counter = us_timer_counter;
    }
}

static void startOffFadeDown(void)
{
    us_timer_counter = 0;
    uint32_t temp_counter = 0;
    int i;

    while (temp_counter <= WHILE_WAIT)
    {
        for(i = LED_CHANNELS-1; i >= 0; i--)
        {
            if(led_brightness[i] > 0)
            {
                // TODO Do i need to add some delta +- times here encase the interrupts are too faaast?
                if(temp_counter == (fade_times[max_brightness - led_brightness[i]] + stair_times[LED_CHANNELS-1-i]))
                {
                    led_brightness[i]--;
                }
            }
        }
        temp_counter = us_timer_counter;
    }
}

static void startOffFadeUp(void)
{
    int i;
    uint32_t temp_counter = 0;
    us_timer_counter = 0;

    while (temp_counter <= WHILE_WAIT)
    {
        for(i = 0; i < LED_CHANNELS; i++)
        {
            if(led_brightness[i] > 0)
            {
                if(temp_counter >= (fade_times[max_brightness - led_brightness[i]] + stair_times[i]))
                {
                    led_brightness[i]--;
                }
            }
        }
        temp_counter = us_timer_counter;
    }
}

static void startOnFadeDown(void)
{
    us_timer_counter = 0;
    uint32_t temp_counter = 0;
    int i;

    while (temp_counter <= WHILE_WAIT)
    {
        for(i = LED_CHANNELS-1; i >= 0; i--)
        {
            if(led_brightness[i] < max_brightness)
            {
                if(temp_counter == (fade_times[led_brightness[i]] + stair_times[LED_CHANNELS-1-i]))
                {
                    led_brightness[i]++;
                }
            }
        }
        temp_counter = us_timer_counter;
    }
}

void waitForGap(void)
{
    us_timer_counter = 0;
    while(us_timer_counter < CONVERT_2_COUNT(GAP_DURATION)){}
}

void waitForGapv2(uint16_t ms)
{
    us_timer_counter = 0;
    while(us_timer_counter < CONVERT_2_COUNT(ms)){}
}


void startFade(direction_e dir)
{
    setTimeOfDay();

    switch(dir)
    {
        case DIR_UP:
            startOnFadeUp();
            waitForGap();
            startOffFadeUp();
        break;
        case DIR_DOWN:
            startOnFadeDown();
            waitForGap();
            startOffFadeDown();
        break;
        default:
            break;
    }
}

int checkToF(direction_e up_down)
{
    uint16_t value = 0;
    if(up_down == DIR_UP){
        vl53l0x_get_measurement(&vl53l0x_bottom, SENSOR_CHAN_PROX, &value);
    }
    if(up_down == DIR_DOWN){
        vl53l0x_get_measurement(&vl53l0x_top, SENSOR_CHAN_PROX, &value);
    }
    return value;
}


void hackPwm(void)
{
    /*setLedBrightness(0,10);
    waitForGap();
    setLedBrightness(0,20);
    waitForGap();
    setLedBrightness(0,30);
    waitForGap();
    setLedBrightness(0,40);
    waitForGap();
    setLedBrightness(0,50);
    waitForGap();
    setLedBrightness(0,60);
    waitForGap();
    setLedBrightness(0,70);
    waitForGap();
    setLedBrightness(0,80);
    waitForGap();
    setLedBrightness(0,90);
    waitForGap();
    setLedBrightness(0,100);
    waitForGap();*/
    int i;
    for(i=0; i<=100; i++)
    {
        setLedBrightness(0,i);
        waitForGapv2(50);
    }
}