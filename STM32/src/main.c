#include "stm32f0xx.h"
#include "stairlight_functions.h"
#include "main.h"

int main (void)
{
    stairlightSetup();

    //startFade(DIR_UP);
    //hackPwm();

    while(1)
    {
        //hackPwm();
        /*if((GPIOB->IDR & GPIO_Pin_8))
        {
            startFade(DIR_UP);
        }
        if((GPIOB->IDR & GPIO_Pin_9))
        {
            startFade(DIR_DOWN);
        }*/
        if(checkToF(DIR_UP))
        {
            startFade(DIR_UP);
        }
    }
}
