#ifndef __MAIN_H
#define __MAIN_H

#include <string.h>
#include "stm32f0xx.h"

typedef enum{
    PARABOLA_FS,
    PARABOLA_SS,
    LINEAR,
}graph_type_e;

typedef struct{
    GPIO_TypeDef* port;
    uint32_t pin;
}led_channel_t;

typedef enum{
    DIR_UP,
    DIR_DOWN,
}direction_e;

typedef enum{
    TIME_DAY,
    TIME_NIGHT,
}time_e;

// True/False things
typedef uint8_t bool;
#define true  1
#define false 0

// ============================= LED Strip configs =============================
#define FADE_DURATION   1000    // The time for one LED to fade on fully (ms)
#define STEPS_DURATION  3000    // The time for all the steps to light up (ms)
#define GAP_DURATION    3500    // The time between the lights being fully on and when the lights start turning off again (ms)
#define MAX_BRIGHTNESS_DAY   100 // Value where the LED brightness should be capped (%)
#define MAX_BRIGHTNESS_NIGHT 20 // Value where the LED brightness should be capped (%)
#define LED_CHANNELS    12      // Number of stairs. Don't change
#define FREQUENCY       400     // The frequency of the PWM signal (Hz) (Tested up to 400Hz)
#define STAIR_FUNCTION  LINEAR
#define FADE_FUNCTION   PARABOLA_SS

// ============================== LED trigger settings ==============================
#define TRIGGER_VL53L0X         // Define to use the TOF sensors to trigger stair lights
//#define TRIGGER_BUTTON          // Define to use push buttons to trigger stair lights
#define VARIABLE_BRIGHTNESS     // Enable multiple brightness profiles

// ============================== Maintainance settings ==============================
//#define ENABLE_DEBUG            // printf via USART
#define ENABLE_WATCH_DAWG       // Enable the watch dog

// ================================ GPIO PIN defines ================================
#define GPIO_I2C_SCL                    GPIO_PinSource8
#define GPIO_I2C_SDA                    GPIO_PinSource9
#define GPIO_ADC_IN                     GPIO_Pin_0
#define GPIO_XSHUT_TOP_PIN              GPIO_Pin_10
#define GPIO_XSHUT_BOTTOM_PIN           GPIO_Pin_11
#define GPIO_TOF_INT_TOP_PIN            GPIO_PinSource1
#define GPIO_TOF_INT_BOTTOM_PIN         GPIO_PinSource2
#define GPIO_PUSH_BUTTONS_TOP_PIN       GPIO_Pin_3
#define GPIO_PUSH_BUTTONS_BOTTOM_PIN    GPIO_Pin_4
#define GPIO_PUSH_BUTTONS_IN            GPIO_PUSH_BUTTONS_TOP_PIN | GPIO_PUSH_BUTTONS_BOTTOM_PIN //GPIO_Pin_8 | GPIO_Pin_9
#define GPIO_PUSH_BUTTONS_OUT           GPIO_Pin_5 //GPIO_Pin_10 | GPIO_Pin_11 // we should rather connect this to high or ground or whatever
#define GPIO_USART_TX                   GPIO_PinSource14 //GPIO_PinSource2 //GPIO_PinSource14

#endif // __MAIN_H