#include "func_buttons.h"

#ifdef TRIGGER_BUTTON

void buttonSetup(void)
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
    /*
    GPIO_InitStructure.GPIO_Pin = GPIO_PUSH_BUTTONS_OUT;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	  	// output
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // output should not have pull up/down
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIOB->BSRR = GPIO_PUSH_BUTTONS_OUT;                // Set High
    */
}

int checkButtonPress(direction_e up_down)
{
    switch(up_down)
    {
        case DIR_UP:
            return (GPIOB->IDR & GPIO_PUSH_BUTTONS_BOTTOM_PIN);
        case DIR_DOWN:
            return (GPIOB->IDR & GPIO_PUSH_BUTTONS_TOP_PIN);
        default:
            return 0;
    }
}

#else

void buttonSetup(void){}
int checkButtonPress(direction_e up_down){return 0;}

#endif // TRIGGER_BUTTON