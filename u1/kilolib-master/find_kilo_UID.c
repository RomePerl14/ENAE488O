#include "kilolib.h"
int blink = 0;

void setup() 
{ 
    if((uint8_t) kilo_uid == 0)
    {
        set_color(RGB(0,0,0));
    }
    else if((uint8_t) kilo_uid == 1)
    {
        set_color(RGB(1,0,0));
    }
    else if((uint8_t) kilo_uid == 2)
    {
        set_color(RGB(0,1,0));
    }
    else if((uint8_t) kilo_uid == 3)
    {
        set_color(RGB(0,0,1));
    }
    else if((uint8_t) kilo_uid == 4)
    {
        set_color(RGB(1,0,1)); //PURPLE
    }
    else if((uint8_t) kilo_uid == 5)
    {
        set_color(RGB(1,1,0)); // YELLOW
    }
    else
    {
        if(blink == 0) {
            set_color(RGB(1, 1, 1));
            blink = 1;
        }
        else if(blink == 1) {
            set_color(RGB(0,0,0));
            blink = 0;
        }
        delay(100);
    }
}


void loop() 
{
    
}


int main() 
{
    kilo_init();
    kilo_start(setup, loop);
    return 0;
}
