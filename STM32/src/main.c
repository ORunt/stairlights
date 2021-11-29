#include "stm32f0xx.h"
#include "stairlight_functions.h"
#include "main.h"

int main (void)
{
    stairlightSetup();
    
    startFade(DIR_UP);

    while(1)
    {
        
    }
}
