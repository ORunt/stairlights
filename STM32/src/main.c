#include "main.h"
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
    }
}
