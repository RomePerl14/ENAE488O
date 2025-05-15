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
uint8_t local_network_binary[8];
uint8_t send_local_network_binary = 0;
uint16_t heartbeat_check[16];
uint8_t num_neighbors = 0;

uint8_t IN_CONTACT_THRESHOLD = 4; // basically, wait roughly 3 seconds before checking
uint32_t start_time;

uint32_t flashing_timer;
flashing_state_t flashing_state = FLASHING;
uint8_t neighbors_network_size_map[16];
uint8_t neighbors_network_size = 0;
uint8_t neighbor_network_binaries[5]; // only 5 kilobots in this thang

uint32_t motion_timer;
uint32_t global_timer;
uint32_t global_number = 0;

void check_heartbeats()
{
    // Check to see if we've lost contact with a kilobot
    uint8_t counter = 0; // counter to keep track of our location
    do // while our counter is less than the number of neighbors we know of
    {
        // This should remove all values in the heartbeat_check that are greater than our threshold value
        if(heartbeat_check[counter] >= IN_CONTACT_THRESHOLD) // if the current neighobrs heartbeat value is greater than the preset threshold, we've lost contact with it
        {
            neighbor_network_binaries[local_network[counter]-1] = 0;
            // if the counter is currently at the very end of the local_network, just set our lost neighbor's values to 0
            if(counter == (num_neighbors-1)) // case for where we're at the very end of the array
            {
                // instead of squashing them, set them to 0
                local_network[counter] = 0; 
                heartbeat_check[counter] = 0;
            }
            else // if it's not at the very end, squash it's values with the neighbor to the right of it in the local_network array, effectively getting rid of it
            {
                for(uint8_t i=counter;i<num_neighbors;i+=1)
                {
                    // This algorithm is not very efficient, since it will repeat n+1 times, where n is the number of kilobots we've lost contact with
                    local_network[counter] = local_network[counter+1]; // shift everything left, squash the current kilobot
                    heartbeat_check[counter] = heartbeat_check[counter+1]; // shift everything left, squash the current kilobot's heartbeat
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
    switch(global_number)
    {
        case 0:
            set_color(RGB(1,1,1)); // WHITE
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

// setup schtuff
void setup() 
{   
    // fill our arrays with zero
    for(uint8_t i=0;i<16;i+=1)
    {
        local_network[i] = 0;
        heartbeat_check[i] = 0;
    }
    for(uint8_t i=0;i<5;i+=1)
    {
        neighbor_network_binaries[i] = 0;
    }
    for(uint8_t i=0;i<8;i+=1)
    {
        local_network_binary[i] = 0;
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
}

// now loop
void loop() {

    if(new_message == 1)
    {
        new_message = 0;
    }

    if((kilo_ticks-start_time) >= 8) // update our heartbeat every half-second (roughly)
    {
        for(uint8_t i=0;i<num_neighbors;i+=1)
        {
            heartbeat_check[i] += 1;
            
        }
        start_time = kilo_ticks;
    }    

    // regardless of if we got a message, check the heartbeats
    check_heartbeats();

    // create local_network_binary
    for(uint8_t i=0;i<8;i+=1)
    {
        local_network_binary[i] = 0; // fill with 0s
    }
    for(uint8_t i=0;i<num_neighbors;i+=1)
    {
        if(local_network[i] != 0)
        {
            local_network_binary[local_network[i]-1] = 1; // get the correct location
        }
    }

    // update our binary 
    local_network_binary[kilo_uid] = 1; // always say that our local network binary is 1
    send_local_network_binary = 0;
    for(uint8_t i=0; i<8; i+=1) 
    {
        send_local_network_binary |= (local_network_binary[i] << i); // store in LSB format
    }
    neighbor_network_binaries[kilo_uid] = send_local_network_binary; // update and send my local network
     
    global_number = 0;
    for(uint8_t neighborN=0;neighborN<num_neighbors;neighborN+=1) // go through each neighbor and check their local network
    {
        uint8_t neighbor_check = 0; // value to store OR'd bit in
        for(uint8_t i=0;i<5;i+=1)
        {
            if(i != neighborN) // only do operations on every network other than the current one we're looking at
            {
                neighbor_check |= neighbor_network_binaries[local_network[neighborN] - 1]; // OR all of the local networks together
            }
        }

        // Now, check the value of the N bit in the binary number

        // Extracting the 5th bit (0-based index)
        uint8_t bit_position = neighborN;

        // Create a mask with only the 5th bit set to 1
        uint8_t mask = 1 << bit_position;

        // Extract the bit using AND and right shift
        unsigned int extracted_bit = (neighbor_check & mask) >> bit_position;

        if(extracted_bit == 1)
        {
            global_number += 1;
        }
    }
    // Finally, condition our global number:
    if(global_number != 0)
    {
        global_number -= 1; // subtract 1
    }

    // check if we have no neighbors, and if so, we're at 0
    if(num_neighbors == 0)
    {
        global_number = 0;
    }

    // get the color
    get_kilo_color();
                                                                                                                                                                                                               
}

message_t *message_tx() 
{
    msg.type = NORMAL;
    // Transmit our kilo_uid
    msg.data[0] = (uint8_t) kilo_uid;
    for(uint8_t i=0;i<5;i+=1)
    {
        msg.data[i+1] = neighbor_network_binaries[i]; // share all of the neighbor network binary numbers
    }
    // Do this thing
    msg.crc = message_crc(&msg);

    return &msg;
}

void message_rx(message_t *m, distance_measurement_t *d) {
    new_message = 1;
    
    // get the distance just in case
    dist = *d;
    cur_distance = estimate_distance(&dist);

    rx_kilo_id = m->data[0]+1; // always add 1 so that no kilo_uid should be less than 1 (i.e. 0 - which we dont want because I'm using it as an empty value place holder for my large ass arrays

    for(uint8_t i=0;i<5;i+=1)
    {
        if(i != kilo_uid)
        {   
            neighbor_network_binaries[i] = m->data[i+1]; // read in everyone's local network (except our own just incase)
        }
    }

    // if we have no neighbors, do some initial setup
    if(num_neighbors == 0)
    {
        num_neighbors = 1; // increment our number of neighbors
        local_network[0] = rx_kilo_id; // save our neighbor's id in the local network
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
        }
        // If we received a message from a new neighbor, add it to the local_network
        else if(check_for_id == 0) // if the kilo_id is NOT in the local_network, add it to our network
        {
            num_neighbors += 1; // increment the number of registered neighbors
            local_network[num_neighbors-1] = rx_kilo_id; // add the new kilo_uid to the end of the list
        }
        else if(check_for_id == 2) // if for some reason we found it twice, squash one of them
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
