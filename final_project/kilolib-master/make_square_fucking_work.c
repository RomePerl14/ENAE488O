#include "kilolib.h"

// declare variables
uint8_t new_message = 0;

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
int current_motion = STOP;

unsigned int count = 0;

unsigned int kilo_made_it_0 = 0;
unsigned int kilo_made_it_1 = 0;
unsigned int kilo_made_it_2 = 0;
unsigned int kilo_made_it_3 = 0;
unsigned int kilo_made_it_4 = 0;
unsigned int kilo_made_it_5 = 0;

int kilo_dist_0;
int kilo_dist_1;
int kilo_dist_2;
int kilo_dist_3;
int kilo_dist_4;
int kilo_dist_5;

uint8_t friend_uid;

message_t msg;

uint8_t kilo_made_it = 0;
uint8_t friend_kilo_made_it = 0;

// FROM TOP - BOTTOM:
// 4 2 0 1 3 5
   
distance_measurement_t dist_measure;

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
    if(kilo_uid == 0)
    {
        set_color(RGB(1,0,0));
    }
    else if(kilo_uid == 1)
    {
        set_color(RGB(0,1,0));
    }
    else if(kilo_uid == 2)
    {
        set_color(RGB(0,0,1));
    }
    else if(kilo_uid == 3)
    {
        set_color(RGB(1,1,0));
    }
    else if(kilo_uid == 4)
    {
        set_color(RGB(1,0,1));
    }
    else if(kilo_uid == 5)
    {
        set_color(RGB(0,1,1));
    }

}


void loop() 
{
    // FROM TOP - BOTTOM:
    // 0 1 2 3 4 5
    if(kilo_made_it == 0)
    {
        if(kilo_uid == 1 || kilo_uid == 3)
        {
            set_motion(FORWARD);
            delay(6000);
            set_motion(LEFT);
            delay(2000);
            set_motion(FORWARD);
            delay(3000);
            set_motion(STOP);
            kilo_made_it = 1;
            set_color(RGB(1,1,1));
        }
    }

}

// read message, and get the distance
void message_rx(message_t *m, distance_measurement_t *d) 
{
    // When I've recieved a message, figure out who it's from
    friend_uid = m->data[0];
    if(friend_uid == 0)
    {
        kilo_dist_0 = estimate_distance(d);
        kilo_made_it_0 = m->data[1];
    }
    else if(friend_uid == 1)
    {
        kilo_dist_1 = estimate_distance(d);
        kilo_made_it_1 = m->data[1];

    }
    else if(friend_uid == 2)
    {
        kilo_dist_2 = estimate_distance(d);
        kilo_made_it_2 = m->data[1];

    }
    else if(friend_uid == 3)
    {
        kilo_dist_3 = estimate_distance(d);
        kilo_made_it_3 = m->data[1];

    }
    else if(friend_uid == 4)
    {
        kilo_dist_4 = estimate_distance(d);
        kilo_made_it_4 = m->data[1];

    }
    else if(friend_uid == 5)
    {
        kilo_dist_5 = estimate_distance(d);
        kilo_made_it_5 = m->data[1];
    }
    new_message = 1;
}

message_t *message_tx() 
{
    // Transmit my ID number
    msg.type = NORMAL;
    msg.data[0] = (uint8_t) kilo_uid; // just incase
    msg.data[1] = (uint8_t) kilo_made_it; // message indicating that this kilobot has made it to their target
    
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
