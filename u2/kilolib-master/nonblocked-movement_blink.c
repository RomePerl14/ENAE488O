#include "kilolib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


uint32_t last_changed;

void setup()
{
    
}

void loop()
{
    if(kilo_ticks > (last_changed + 64))
    {
        last_changed = kilo_ticks; // To remember the current time... I guess?
        set_color(RGB(1,1,0));
        delay(100);
        set_color(RGB(0,0,0));
    }
}

int main()
{
    kilo_init();
    kilo_start(setup, loop);

    return 0;
}
