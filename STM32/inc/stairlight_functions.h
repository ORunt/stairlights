#include "stm32f0xx.h"

typedef enum{
    PARABOLA_FS,
    PARABOLA_SS,
    LINEAR,
}graph_type_e;

// LED Strip configs
#define FADE_DURATION   1000    // The time for one LED to fade on fully (ms)
#define STEPS_DURATION  5000    // The time for all the steps to light up (ms)
#define GAP_DURATION    2000    // The time between the lights being fully on and when the lights start turning off again (ms)
#define MAX_BRIGHTNESS_DAY   80 // Value where the LED brightness should be capped (%)
#define MAX_BRIGHTNESS_NIGHT 30 // Value where the LED brightness should be capped (%)
#define LED_CHANNELS    12      // Number of stairs. Don't change
#define FREQUENCY       400     // The frequency of the PWM signal (Hz) (Tested up to 400Hz)
#define STAIR_FUNCTION  PARABOLA_SS
#define FADE_FUNCTION   PARABOLA_SS

// LED trigger settings
#define TRIGGER_VL53L0X         // Define to use the TOF sensors to trigger stair lights
//#define TRIGGER_BUTTON          // Define to use push buttons to trigger stair lights

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

void stairlightSetup(void);
void startFade(direction_e dir);
void hackPwm(void);
int checkToF(direction_e up_down);