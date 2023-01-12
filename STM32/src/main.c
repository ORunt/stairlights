#include "main.h"
#include "func_debug.h"
#include "func_buttons.h"
#include "func_ldr.h"
#include "func_led.h"
#include "func_vl53l0x.h"
#include "func_watchdog.h"

extern bool dbg_error_state;

bool checkTrigger(direction_e dir)
{
    bool result = 0;
    result |= (bool)checkButtonPress(dir);
    result |= (bool)checkToF(dir);
    watchdogPet();
    return result;
}

void pauseTrigger(bool pause)
{
    startTof(!pause);
}

int main (void)
{
    uint8_t main_loop = 0;
    ledSetup();
    ldrSetup();
    buttonSetup();
    vl53l0xSetup();
    printfSetup();
    watchdogSetup();

    while(1)
    {
        if(checkTrigger(DIR_UP))
        {
            pauseTrigger(true);
            startFade(DIR_UP);
            pauseTrigger(false);
        }
        if(checkTrigger(DIR_DOWN))
        {
            pauseTrigger(true);
            startFade(DIR_DOWN);
            pauseTrigger(false);
        }
        if(dbg_error_state){
            main_loop ^= 1;
            if(main_loop){
                DBG_ERR_CHK(1,12);
            }
            else{
                DBG_ERR_CHK_CLR(1,12);
            }
        }
    }
}
