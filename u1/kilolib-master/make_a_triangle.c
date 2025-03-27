#include "kilolib.h"

// declare variables
uint8_t new_message = 0;
uint8_t dist = 0;

unsigned int friend_uid;
message_t msg;
   

distance_measurement_t dist_measure;

void setup() 
{ 
    msg.type = NORMAL;
    msg.crc = message_crc(&msg);
}


void loop() 
{
    // If I've gotten a new message
    if (new_message == 1) 
    {
        new_message = 0;
        dist = estimate_distance(&dist_measure);

        if(friend_uid == 1000)
        {
            // if(dist <= 50) 
            // {   
                set_color(RGB(0,0,1));
            // }
        }
        else if(friend_uid == 1001)
        {
            // if(dist <= 50) /
            // {   
                set_color(RGB(0,1,0));
            // }
        }
        else if(friend_uid == 1002)
        {
            // if(dist <= 50) 
            // {   
                set_color(RGB(1,0,0));
            // }
        }
        else
        {
            // if(dist <= 50) 
            // {   
                set_color(RGB(1,1,1));
            // }
        }
        // delay(100);
        // set_color(RGB(0,0,0));
    }
    delay(100);
    set_color(RGB(0,0,0));

    if(kilo_uid == 1000)
    {
        delay(100);
        set_color(RGB(1,0,0));
    }
    else if(kilo_uid == 1001)
    {
        delay(100);
        set_color(RGB(0,0,1));
    }
    delay(100);
    set_color(RGB(0,0,0));
}

// update even an odd with message reception
void message_rx(message_t *m, distance_measurement_t *d) 
{
    // When I've recieved a message, figure out who it's from
    friend_uid = m->data[0];
    dist_measure = *d;
    new_message = 1;
}

message_t *message_tx() 
{
    // Transmit my ID number
    msg.data[0] = kilo_uid;
    return &msg;
}

int main() 
{
kilo_init();
kilo_message_rx = message_rx;
kilo_message_tx = message_tx;
kilo_start(setup, loop);
return 0;
}
