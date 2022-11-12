#include "func_ldr.h"
#include "graph_lookup_table.h"

#ifdef VARIABLE_BRIGHTNESS

uint16_t ldr_measurement = 0;

#define ADC_DAY_TIME_THRESHOLD  400 // Max is 4096

void ldrSetup(void)
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
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
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


static uint16_t getAdcConversion(void)
{
    ADC_StartOfConversion(ADC1);

    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    /* Get ADC1 converted data */
    return ldr_measurement = ADC_GetConversionValue(ADC1);
}

static time_e getTimeOfDay(void)
{
    return (getAdcConversion() > ADC_DAY_TIME_THRESHOLD) ? TIME_DAY : TIME_NIGHT;
}

#else

void ldrSetup(void){}

#endif // VARIABLE_BRIGHTNESS

time_e setTimeOfDay(uint32_t * fade_times, uint8_t * max_brightness)
{
#ifdef VARIABLE_BRIGHTNESS
    time_e time_of_day = getTimeOfDay();
#else
    time_e time_of_day = TIME_DAY;
#endif
    switch(time_of_day)
    {
        case TIME_DAY:
            fade_times = &fade_times_day[0];
            *max_brightness = MAX_BRIGHTNESS_DAY;
        break;
        case TIME_NIGHT:
            fade_times = &fade_times_night[0];
            *max_brightness = MAX_BRIGHTNESS_NIGHT;
        break;
        default: break;
    }

    return time_of_day;
}