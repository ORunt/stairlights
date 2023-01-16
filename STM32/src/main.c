#include "main.h"
#include "func_debug.h"
#include "func_buttons.h"
#include "func_ldr.h"
#include "func_led.h"
#include "func_vl53l0x.h"
#include "func_watchdog.h"


bool checkTrigger(direction_e dir, uint16_t * err_state)
{
    bool result = 0;
    result |= (bool)checkButtonPress(dir);
    result |= (bool)checkToF(dir, err_state);
    watchdogPet();
    return result;
}

void pauseTrigger(bool pause)
{
    startTof(!pause);
}

int main (void)
{
    uint16_t err = 0;
    ledSetup();
    ldrSetup();
    buttonSetup();
    vl53l0xSetup();
    printfSetup();
    watchdogSetup();

    while(1)
    {
        if(checkTrigger(DIR_UP, &err))
        {
            pauseTrigger(true);
            startFade(DIR_UP);
            pauseTrigger(false);
        }
        if(checkTrigger(DIR_DOWN, &err))
        {
            pauseTrigger(true);
            startFade(DIR_DOWN);
            pauseTrigger(false);
        }

        // Klap that watch dog
        if(err)
        {
            while(1){}
        }
    }
}
