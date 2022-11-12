#include "main.h"
#include "func_debug.h"
#include "func_buttons.h"
#include "func_ldr.h"
#include "func_led.h"
#include "func_vl53l0x.h"

bool checkTrigger(direction_e dir)
{
    bool result = 0;
    result |= (bool)checkButtonPress(dir);
    result |= (bool)checkToF(dir);
    return result;
}

void pauseTrigger(bool pause)
{
    startTof(!pause);
}

int main (void)
{
    ledSetup();
    ldrSetup();
    buttonSetup();
    vl53l0xSetup();
    printfSetup();
    
    //int d_cnt = 0;

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
        /*int i = 0;
        for(i=0; i<1000000; i++){
            if (i == (1000000-1)){
                d_cnt++;
                DEBUG_PRINT("Count mode x: %d, y: %d\n\r", d_cnt, d_cnt *2);
            }
        }*/
    }
}
