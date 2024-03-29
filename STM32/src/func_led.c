#include "func_led.h"
#include "func_ldr.h"
#include "func_watchdog.h"
#include "graph_lookup_table.h"

// PSC = ceil((64Mhz x 10us / 160) - 1)
// PSC = ceil((8Mhz x 10us / 40) - 1)
#define CONVERT_2_COUNT(ms) ((ms) * (FREQUENCY/10))
#define WHILE_WAIT          CONVERT_2_COUNT(STEPS_DURATION + FADE_DURATION)
#define PULSE_PERIOD        (10000/FREQUENCY)      // The pulse period of the main timer in (us)
#define AC_DIM_PRESCALER	1
#define ARR                 (PULSE_PERIOD * 16)

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
                                                    {GPIOB, GPIO_Pin_7},
                                                    {GPIOB, GPIO_Pin_5}
                                                };

volatile uint8_t led_brightness[LED_CHANNELS] = {0};
uint8_t tim3_counter = 0;  // increments in 10us steps
volatile uint32_t us_timer_counter = 0;
const uint32_t * fade_times = &fade_times_day[0];
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


static void TIM_EnableInterrupt(FunctionalState state)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Set the TIM3 global Interrupt State */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = state;
    NVIC_Init(&NVIC_InitStructure);

    /* TIM Interrupts enable */
    TIM_ITConfig(TIM3, TIM_IT_CC1, state);

    /* TIM3 enable counter */
    TIM_Cmd(TIM3, state);

    /* TIM3 clear any pending interrupts */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
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

    // Don't start up the timer or interrupt yet.
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}


void ledSetup(void)
{
    CLK_Config();
    TIM_Config();
    GPIO_Config();
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


static void setLedBrightness(uint8_t channel, uint8_t brightness)
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
        watchdogPet();
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
                if(temp_counter >= (fade_times[max_brightness - led_brightness[i]] + stair_times[LED_CHANNELS-1-i]))
                {
                    led_brightness[i]--;
                }
            }
        }
        temp_counter = us_timer_counter;
        watchdogPet();
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
        watchdogPet();
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
                if(temp_counter >= (fade_times[led_brightness[i]] + stair_times[LED_CHANNELS-1-i]))
                {
                    led_brightness[i]++;
                }
            }
        }
        temp_counter = us_timer_counter;
        watchdogPet();
    }
}

static void forceAllLightsOff(void)
{
    int i;
    memset((void*)led_brightness, 0, LED_CHANNELS);
    for(i=0; i<LED_CHANNELS; i++)
    {
        GPIO_ResetBits(led_channels[i].port, led_channels[i].pin);
    }
}

void waitForGap(void)
{
    us_timer_counter = 0;
    while(us_timer_counter < CONVERT_2_COUNT(GAP_DURATION)){watchdogPet();}
}

void waitForGapv2(uint16_t ms)
{
    us_timer_counter = 0;
    while(us_timer_counter < CONVERT_2_COUNT(ms)){watchdogPet();}
}


void hackPwm(void)
{
    int i;
    for(i=0; i<=100; i++)
    {
        setLedBrightness(0,i);
        waitForGapv2(50);
    }
}




void startFade(direction_e dir)
{
    setTimeOfDay(fade_times, &max_brightness);
    TIM_EnableInterrupt(ENABLE);
    switch(dir)
    {
        case DIR_UP:
            startOnFadeUp();
            waitForGap();
            startOffFadeUp();
            forceAllLightsOff();
        break;
        case DIR_DOWN:
            startOnFadeDown();
            waitForGap();
            startOffFadeDown();
            forceAllLightsOff();
        break;
        default:
            break;
    }
    TIM_EnableInterrupt(DISABLE);
}