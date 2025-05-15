/*
    ENAE448O MULTI-ROBOT SWARMS
    Final Competition Project
    Group 3
    Romeo Perlstein, Benjamin Tebeest, Oluwatumininu Olanrewaju

    code by:
    Romeo, Ben

    Holy Moley, global communication is rough, how I wish the earth was flat!
*/

#include "kilolib.h"
#include <math.h>

// Enumerator to switch between our flashing states
typedef enum
{
    FLASHING,
    NOT_FLASHING,
} flashing_state_t;

// Declare motion variable type (from orbit-planet.c.c)
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

// Variable definitions
distance_measurement_t dist; // distance measurement
uint8_t rx_kilo_id; // received kilobot ID
message_t msg; // message variable
uint8_t cur_distance = 0; // current distance
uint8_t new_message = 0; // indicate if we got a new message

//---- LOCAL NETWORK VARIABLES ----//
// Array that contains are local network - each index contains a kilo_uid that we've received from a neighbor (index 0 = first neighbor found, index END = most recent new neighbor)
uint16_t local_network[16]; // currently, we're only able to operator with up to 16 neighbors due to memory? We could also dynamically modify the size, but thats out of the scope for this I think
// Array that contains the heartbeat value of each neighbor, in the same order as the local_network[] array (indicies are 1:1)
uint16_t heartbeat_check[16]; // same deal as above size-wise, could dynamically allocate but its easier to do this
uint8_t num_neighbors = 0; // Keep track of how many neighbors we've connected to 
// Array to store our local network in binary representation (due to array indexing, represented in MSB)
uint8_t local_network_binary[8]; // since each data index in our message variable is one byte, limit the array size to 8 elements (like bits)
uint8_t send_local_network_binary = 0; // Variable to store the binary representation of our local array

//---- TIMIING ----//
uint8_t IN_CONTACT_THRESHOLD = 4; // basically, wait roughly 4 seconds before dropping an inactive neighbor
uint32_t start_time; // start time to keep track of how long we've been waiting for a neighbors contact
uint32_t motion_timer; // timer so we can move for a certain length of time at a certain interval without needing delay()
uint32_t global_timer; // timer so that we can wait X seconds before starting motion 

//---- NEIGHBORS ----//
// array to store the binary representations of every kilobot in the global network's local network
uint8_t neighbor_network_binaries[5]; // only 5 kilobots in this thang
uint32_t global_number = 0; // used to represent the number of kilobots in the network

//-- MOTION --//
motion_t cur_motion = STOP; // current motion state


// Function to check the heartbeat values of each neighbor, and figure out if we should drop em (they're no longer in the network)
void check_heartbeats()
{
    // Check to see if we've lost contact with a kilobot
    uint8_t counter = 0; // counter to keep track of our location in the local_network[] array
    // while our counter is less than the number of neighbors we know of, check to see if any of our neighbors have exceeded the threshold value
    // and if so, drop em from the array (and thus, our local network)
    do 
    {
        // Check if the current neighbor's heartbeat has surpassed the threshold
        if(heartbeat_check[counter] >= IN_CONTACT_THRESHOLD) // if the current neighobrs heartbeat value is greater than the preset threshold, we've lost contact with it
        {
            neighbor_network_binaries[local_network[counter]-1] = 0;
            // if the counter is currently at the very end of the local_network, just set our lost neighbor's values to 0
            if(counter == (num_neighbors-1))
            {
                // instead of squashing them, set them to 0
                local_network[counter] = 0; 
                heartbeat_check[counter] = 0;
            }
            // if it's not at the very end, squash it's values with the neighbor to the right of it in the local_network array, effectively getting rid of it
            else 
            {
                // For every element (starting at the current count), take the value and shift it to the left, squashing the values at the current count
                for(uint8_t i=counter;i<num_neighbors;i+=1)
                {
                    // This algorithm is not very efficient, since it will repeat n+1 times, where n is the number of kilobots we've lost contact with
                    local_network[counter] = local_network[counter+1]; // shift everything left, squash the current kilobot
                    heartbeat_check[counter] = heartbeat_check[counter+1]; // shift everything left, squash the current kilobot's heartbeat
                }
            }
            counter = -1; // reset back to the beginning to check through the array
            num_neighbors -= 1; // decrement the number of neighbors
        }
        counter += 1;
    }
    while (counter < num_neighbors); // ending condition
}

// figure out the current color we need given the number of neighbors we have
void get_kilo_color()
{
    // switch condition
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

        default: // any other case, I guess just turn off the LED lol
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
    global_timer = kilo_ticks;
    set_color(RGB(1,1,1));
}

// now loop
void loop() {

    // check if we got a new message, and if we did, set the new message flag to 0 (so it feels important)
    if(new_message == 1)
    {
        new_message = 0;
    }

    if((kilo_ticks-start_time) >= 16) // update our heartbeat every half-second (roughly)
    {
        // every half-second, increment our heartbeat value
        for(uint8_t i=0;i<num_neighbors;i+=1)
        {
            heartbeat_check[i] += 1;
        }
        start_time = kilo_ticks; // reset our timer
    }    

    // Regardless of whether we got a message, check our heartbeats
    check_heartbeats();

    // Create the binary version of our local network based on the results of the 'check_heartbeats()' function
    for(uint8_t i=0;i<8;i+=1)
    {
        local_network_binary[i] = 0; // First, fill with 0s
    }
    for(uint8_t i=0;i<num_neighbors;i+=1)
    {
        if(local_network[i] != 0) // If we have a neighbor
        {
            local_network_binary[local_network[i]-1] = 1; // Set it's value to 1 in the correct location of the local_network_binary[] array
        }
    }

    // Now, update our binary info
    local_network_binary[kilo_uid] = 1; // always say that our local network binary is 1
    send_local_network_binary = 0; // Reset our variable that contains the binary version of our local network
    for(uint8_t i=0; i<8; i+=1) 
    {
        send_local_network_binary |= (local_network_binary[i] << i); // store in LSB format
    }
    neighbor_network_binaries[kilo_uid] = send_local_network_binary; // Update the neighbor_network_binaries[] array with our updated binary local_network

    // After roughly 3 seconds, check our message variables and see what our state is
    if((kilo_ticks-global_timer) >= (32*3))
    {   
        // Reset our global number variable
        global_number = 0;
        // Go through EACH neighbor that exists, get it's locally stored niehgbor network, and use it to figure out how many nodes are in the global network,
        // and if anyone as left since the last time we've checked
        for(uint8_t neighborN=0;neighborN<5;neighborN+=1) // go through each neighbor and check their local network
        {
            uint8_t neighbor_check = 0; // temp variable to store our binary value after our OR operations

            // For each of the nodes in the network, or them together
            for(uint8_t i=0;i<5;i+=1)
            {
                if(i != neighborN) // only do operations on every network other than the current one we're looking at
                {
                    neighbor_check |= neighbor_network_binaries[i]; // OR all of the local networks together
                }
            }

            // Now, check the value of the N bit in the binary number
            // Extract the Nth position of the binary number (i.e., the value representing the current node we're looking at)
            uint8_t bit_position = neighborN;

            // Create a mask with only the Nth bit set to 1
            uint8_t mask = 1 << bit_position;

            // Extract the bit using AND and right shift
            unsigned int extracted_bit = (neighbor_check & mask) >> bit_position;

            // If the extraced bit is 1, then the kilobot still exists in the network
            if(extracted_bit == 1)
            {
                global_number += 1; // Increment our global number
            }
        }
        // Finally, condition our global number:
        if(global_number != 0)
        {
            global_number -= 1; // subtract 1 (we incremented for our own current node as well, so we don't want to include that)
        }

        // check if we have no neighbors, and if so, we're at 0
        if(num_neighbors == 0)
        {
            global_number = 0;
        }

        // Get the kilo_color by checking the global network node number
        get_kilo_color();

        // RESET all of our arrays
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
        global_timer = kilo_ticks;
    }
                                                                                                                                                                                                               
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
        else if(check_for_id > 1)
        {
            for(uint8_t i=index;i<num_neighbors;i+=1)
            {
                local_network[i] = local_network[i+1]; // shift everything left, squash the current kilobot
                heartbeat_check[i] = heartbeat_check[i+1]; // shift everything left, squash the current kilobot's heartbeat
            }
        }

        // double check our number of neighbors aligns with the values in our local_network[] array
        uint8_t num_neighbors_check = 0;
        for(uint8_t i=0;i<16;i+=1)
        {
            if(local_network[i] != 0) // if we have a neighbor, note it
            {
                num_neighbors_check+=1; // iterate up
            }
        }
        // If the check is not the same as the num_neighbors we just found above, pick the check
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
