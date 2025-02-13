#include "kilolib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


uint32_t last_changed;
int state = 0;
int state_changed = 0;

// Constants for motion handling function (from simple motion)
#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

int current_motion = STOP;

// Function to handle motion.
void set_motion(int new_motion)
{
    // Only take an action if the motion is being changed.
    if (current_motion != new_motion)
    {
        current_motion = new_motion;
        
        if (current_motion == STOP)
        {
            set_motors(0, 0);
        }
        else if (current_motion == FORWARD)
        {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (current_motion == LEFT)
        {
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (current_motion == RIGHT)
        {
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

void setup()
{
    set_motion(STOP); // redundant
}

void loop()
{
    if(kilo_ticks > (last_changed + 64))
    {
        last_changed = kilo_ticks; // To remember the current time... I guess?
        state_changed = 1;
        set_color(RGB(1,1,0));
        delay(100);
        set_color(RGB(0,0,0));
    }
    if(state_changed == 1)
    {
        state_changed = 0;
        switch (state)
        {
            case 0:
                set_color(RGB(0,1,0));
                set_motion(FORWARD);
                state = 1;
                break;
            case 1:
                set_color(RGB(1,0,0));
                set_motion(LEFT);
                state = 2;
                break;
            case 2:
                set_color(RGB(0,0,1));
                set_motion(RIGHT);
                state = 0;
                break;
            default:
                set_motion(STOP); // again, redundant most likely
                break;
        }
    }
}

int main()
{
    kilo_init();
    kilo_start(setup, loop);

    return 0;
}
