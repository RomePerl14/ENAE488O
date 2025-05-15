/*
    ENAE448O MULTI-ROBOT SWARMS
    Final Competition Project
    Group 3
    Romeo Perlstein, Benjamin Tebeest, Oluwatumininu Olanrewaju

    code by:
    Romeo, Ben

    Dr. Otte, I apologize for my poor performance as a student!
*/

#include "kilolib.h"
#include <math.h>

typedef enum
{
    FLASHING,
    NOT_FLASHING,
} flashing_state_t;

// declare motion variable type (from orbit-planet.c.c)
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

distance_measurement_t dist; // distance measurement
uint8_t rx_kilo_id; // received kilobot ID
message_t msg; // message variable
uint8_t cur_distance = 0;
uint8_t new_message = 0;

uint16_t local_network[16]; // currently, we're only able to operator with up to 16 neighbors due to memory? That must mean theres a more efficient way to do this lol
uint16_t heartbeat_check[16];
uint8_t num_neighbors = 0;

uint8_t IN_CONTACT_THRESHOLD = 4; // basically, wait roughly 3 seconds before checking
uint32_t start_time;

uint32_t flashing_timer;
flashing_state_t flashing_state = FLASHING;
uint8_t neighbors_network_size_map[16];
uint8_t neighbors_network_size = 0;

motion_t cur_motion = STOP; // current motion state
uint32_t motion_timer;
uint32_t global_timer;

// function to set new motion (from orbit-planet.c.c)
void set_motion(motion_t new_motion) {
    if (cur_motion != new_motion) {
        cur_motion = new_motion;
        switch(cur_motion) {
            case STOP:
                set_motors(0,0);
                break;
            case FORWARD:
                spinup_motors();
                set_motors(kilo_straight_left, kilo_straight_right);
                break;
            case LEFT:
                spinup_motors();
                set_motors(kilo_turn_left, 0); 
                break;
            case RIGHT:
                spinup_motors();
                set_motors(0, kilo_turn_right); 
                break;
        }
    }
}

void check_heartbeats()
{
    // Check to see if we've lost contact with a kilobot
    uint8_t counter = 0; // counter to keep track of our location
    do // while our counter is less than the number of neighbors we know of
    {
        // This should remove all values in the heartbeat_check that are greater than our threshold value
        if(heartbeat_check[counter] >= IN_CONTACT_THRESHOLD) // if the current neighobrs heartbeat value is greater than the preset threshold, we've lost contact with it
        {
            // if the counter is currently at the very end of the local_network, just set our lost neighbor's values to 0
            if(counter == (num_neighbors-1)) // case for where we're at the very end of the array
            {
                // instead of squashing them, set them to 0
                local_network[counter] = 0; 
                heartbeat_check[counter] = 0;
                neighbors_network_size_map[counter] = 0;
            }
            else // if it's not at the very end, squash it's values with the neighbor to the right of it in the local_network array, effectively getting rid of it
            {
                for(uint8_t i=counter;i<num_neighbors;i+=1)
                {
                    // This algorithm is not very efficient, since it will repeat n+1 times, where n is the number of kilobots we've lost contact with
                    local_network[counter] = local_network[counter+1]; // shift everything left, squash the current kilobot
                    heartbeat_check[counter] = heartbeat_check[counter+1]; // shift everything left, squash the current kilobot's heartbeat
                    neighbors_network_size_map[counter] = neighbors_network_size_map[counter+1];
                }
            }
            counter = -1; // reset back to the beginning to check through the array ()
            num_neighbors -= 1; // decrement the number of neighbors
        }
        counter += 1;
    }
    while (counter < num_neighbors);
}

void get_kilo_color()
{
    switch(num_neighbors)
    {
        case 0:
            set_color(RGB(1,1,1)); // WHITE
            set_motion(STOP);
            break;
        case 1:
            set_color(RGB(1,0,0)); // RED
            break;
        case 2:
            set_color(RGB(1,1,0)); // YELLOW
            break;

        case 3:
            set_color(RGB(0,1,0)); // GREEN
            break;

        case 4:
            set_color(RGB(0,0,1)); // BLUE
            break;

        case 5:
            break;
            set_color(RGB(0,1,1)); // TEAL

        case 6:
            set_color(RGB(1,0,1)); // PURPLE
            break;

        default: // any other case, I guess just turn off the LED
            set_color(RGB(0,0,0));
            break;
    }
}

void check_flashing_state()
{
    switch(flashing_state)
    {
        case FLASHING:
            if((kilo_ticks-flashing_timer) == 32) //&& (kilo_ticks-flashing_timer) < 64)
            {
                set_color(RGB(0,0,0));
            }
            else if((kilo_ticks-flashing_timer) == 64)
            {
                flashing_timer = kilo_ticks;
                get_kilo_color();
            }
            break;
        case NOT_FLASHING:
            get_kilo_color();
            break;
    }
}

void check_movement_state()
{
    switch(flashing_state)
    {
        case FLASHING:
            if((kilo_ticks-motion_timer) == 64)
            {
                set_motion(STOP);
            }
            else if((kilo_ticks-motion_timer) == 96)
            {
                if(num_neighbors != 0)
                {
                    set_motion(FORWARD);
                }   
                motion_timer = kilo_ticks;
            }
            break;
        case NOT_FLASHING:
            set_motion(STOP);
    }
}

void compare_cardinality()
{
    // Now, check out status compared to our neighbors
    uint8_t smallest_size = 17;
    if(neighbors_network_size_map[0] != 0)
    {
        smallest_size = neighbors_network_size_map[0];
    }
    for(uint8_t i=1;i<num_neighbors;i+=1)
    {
        if(neighbors_network_size_map[i] != 0)
        {
            if(neighbors_network_size_map[i] < smallest_size)
            {
                smallest_size = neighbors_network_size_map[i];
            }
        }
        
    }
    // if(num_neighbors < smallest_size)
    if(num_neighbors <= smallest_size)
    {
        switch(flashing_state)
        {
            case NOT_FLASHING:
                flashing_state = FLASHING;
                flashing_timer = kilo_ticks;
                motion_timer = kilo_ticks;
                break;
            case FLASHING:
                break;
        }
    }
    else
    {
        flashing_state = NOT_FLASHING;
    }
}

// setup schtuff
void setup() 
{   
    // fill our arrays with zero
    for(uint8_t i=0;i<16;i+=1)
    {
        local_network[i] = 0;
        heartbeat_check[i] = 0;
        neighbors_network_size_map[i] = 0;
    }
    
    // set up message
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
    
    start_time = kilo_ticks;
    flashing_timer = kilo_ticks;
    motion_timer = kilo_ticks;
    global_timer = kilo_ticks;
}

// now loop
void loop() {

    if(new_message == 1)
    {
        new_message = 0;
    }

    if((kilo_ticks-start_time) >= 32) // update our heartbeat every half-second (roughly)
    {
        for(uint8_t i=0;i<num_neighbors;i+=1)
        {
            heartbeat_check[i] += 1;
        }
        start_time = kilo_ticks;
    }    
     

    // regardless of if we got a message, check the heartbeats
    check_heartbeats();

    compare_cardinality();

    check_flashing_state();

    if((kilo_ticks-global_timer) > (32*3))
    {
        check_movement_state();
    }
    
    // get_kilo_color();
                                                                                                                                                                                                               
}

message_t *message_tx() 
{
    msg.type = NORMAL;
    // Transmit our kilo_uid
    msg.data[0] = (uint8_t) kilo_uid;
    msg.data[1] = (uint8_t) num_neighbors;
    // Do this thing
    msg.crc = message_crc(&msg);

    return &msg;
}

void message_rx(message_t *m, distance_measurement_t *d) {
    new_message = 1;
    
    // get the distance just in case
    dist = *d;
    cur_distance = estimate_distance(&dist);

    // get the kilo_uid
    if(m->data[0]+1 < 65535)
    {
        rx_kilo_id = m->data[0]+1; // always add 1 so that no kilo_uid should be less than 1 (i.e. 0 - which we dont want because I'm using it as an empty value place holder for my large ass arrays
    }
    else
    {
        return; // just ignore it - I'm doing this because I don't want to use an <int> array and want to use a <uint> array because I'm lazy and also trying to be memory efficient
    }
    
    // get the current sender's network size
    neighbors_network_size = m->data[1];

    // if we have no neighbors, do some initial setup
    if(num_neighbors == 0)
    {
        num_neighbors = 1; // increment our number of neighbors
        local_network[0] = rx_kilo_id; // save our neighbor's id in the local network
        neighbors_network_size_map[0] = neighbors_network_size;
    }
    else
    {
        uint8_t check_for_id = 0; // flag to check if we've already contacted this kilobot
        uint8_t index = 0; // index of the kilobot in our local_network array

        // First, check our array and see if the received kilobot_id is in the local_network array
        for(uint8_t i=0;i<num_neighbors;i+=1)
        {
            if(rx_kilo_id == local_network[i]) // check to see if our current received kilo_id is in the local stored network
            {
                check_for_id += 1; // if it is, flip a flag
                index = i; // save the location that it's in for use lator
            } 
        }

        // If we received a message from an already contacted neighbor
        if(check_for_id == 1) // if we flipped the flag
        {
            heartbeat_check[index] = 0; // set our heartbeat to 0 as we've received a heartbeat
            neighbors_network_size_map[index] = neighbors_network_size; // get the known neighbors network size
        }
        // If we received a message from a new neighbor, add it to the local_network
        else if(check_for_id == 0) // if the kilo_id is NOT in the local_network, add it to our network
        {
            num_neighbors += 1; // increment the number of registered neighbors
            local_network[num_neighbors-1] = rx_kilo_id; // add the new kilo_uid to the end of the list
        }
        else if(check_for_id > 1)
        {
            for(uint8_t i=index;i<num_neighbors;i+=1)
            {
                local_network[i] = local_network[i+1]; // shift everything left, squash the current kilobot
                heartbeat_check[i] = heartbeat_check[i+1]; // shift everything left, squash the current kilobot's heartbeat
            }
        }

        uint8_t num_neighbors_check = 0;
        for(uint8_t i=0;i<16;i+=1)
        {
            if(local_network[i] != 0)
            {
                num_neighbors_check+=1;
            }
        }
        if(num_neighbors != num_neighbors_check)
        {
            num_neighbors = num_neighbors_check;
        }
    }

}

// Do main things
int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    kilo_start(setup, loop);

    return 0;
}
