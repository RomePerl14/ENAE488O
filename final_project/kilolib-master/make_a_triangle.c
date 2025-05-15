#include "kilolib.h"

// declare variables
uint8_t new_message = 0;

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
int current_motion = STOP;

int distance_from_friend;

unsigned int count;

uint8_t friend_uid;

message_t msg;

uint8_t kilo_made_it = 0;
   

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
    msg.type = NORMAL;
    msg.data[0] = kilo_uid;
    msg.data[1] = 0;
    msg.data[2] = 0;
    msg.data[3] = 0;
    msg.data[4] = 0;
    msg.data[5] = 0;
    msg.data[6] = 0;
    msg.data[7] = 0;
    msg.crc = message_crc(&msg);
    if(kilo_uid == 1)
    {
        set_color(RGB(1,0,0));
    }
    else if(kilo_uid == 2)
    {
        set_color(RGB(0,1,0));
    }
    else if(kilo_uid == 3)
    {
        set_color(RGB(0,0,1));
    }

}


void loop() 
{
    if(new_message == 1)
    {
        new_message = 0;
        // get the current distance from the message
        if(kilo_uid == 2)
        {
            if(kilo_made_it == 0)
            {
                set_color(RGB(0,1,0));
                if(distance_from_friend < 80)
                {
                    set_color(RGB(0,1,0));
                    set_motion(FORWARD);
                }
                else
                {
                    set_motion(STOP);
                    set_color(RGB(1,1,1));
                    kilo_made_it = 1;
                }
            }
        }
    }
}
    

// read message, and get the distance
void message_rx(message_t *m, distance_measurement_t *d) 
{
    // When I've recieved a message, figure out who it's from
    friend_uid = m->data[0];
    if(friend_uid == 3)
    {
        distance_from_friend = estimate_distance(d);
    }
    
    new_message = 1;
}

message_t *message_tx() 
{
    // Transmit my ID number
    msg.type = NORMAL;
    msg.data[0] = (uint8_t) kilo_uid; 
    msg.crc = message_crc(&msg);

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
