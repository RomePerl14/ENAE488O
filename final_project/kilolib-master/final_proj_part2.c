/*
    ENAE448O MULTI-ROBOT SWARMS
    Final Competition Project
    Group 3
    Romeo Perlstein, Benjamin Tebeest, Oluwatumininu Olanrewaju

    code by:
    Romeo

    Dr. Otte, I apologize for my poor performance as a student!
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

//---- TIMIING ----//
uint8_t IN_CONTACT_THRESHOLD = 4; // basically, wait roughly 4 seconds before dropping an inactive neighbor
uint32_t start_time; // start time to keep track of how long we've been waiting for a neighbors contact
uint32_t motion_timer; // timer so we can move for a certain length of time at a certain interval without needing delay()
uint32_t global_timer; // timer so that we can wait X seconds before starting motion 
uint32_t flashing_timer; // timer so we can flash at a specific interval without needing to use delay()

//---- FLASHING ----//
flashing_state_t flashing_state = FLASHING; // flashing state variable (intitialize to FLASHING on start)
// An array to store the number of neighbors connected to a neighbor, i.e. if we are node1 in contact with node2, we want to store how many neighbors node2 has (node2 tells us)
uint8_t neighbors_network_size_map[16]; // indicies are mapped 1:1 with the local_network[] array, similar to above
uint8_t neighbors_network_size = 0; // size of our received neighbors local network

//-- MOTION --//
motion_t cur_motion = STOP; // current motion state

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
            // if the counter is currently at the very end of the local_network, just set our lost neighbor's values to 0
            if(counter == (num_neighbors-1))
            {
                // instead of squashing them, set them to 0
                local_network[counter] = 0; 
                heartbeat_check[counter] = 0;
                neighbors_network_size_map[counter] = 0; // reset the size of this neighbor as well
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
                    neighbors_network_size_map[counter] = neighbors_network_size_map[counter+1]; // shift everything to the left, squash the current kilobots neighbor size
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

// Check our current flashing state and decide what action we need to take depending on it
void check_flashing_state()
{
    switch(flashing_state)
    {
        // If we're flashing
        case FLASHING:
            // If its been roughly 32 ticks since our last reset
            if((kilo_ticks-flashing_timer) == 32)
            {
                set_color(RGB(0,0,0)); // set color to zero
            }
            // if its been roughly 64 ticks since our last reset
            else if((kilo_ticks-flashing_timer) == 64)
            {
                // Reset our flashing timer
                flashing_timer = kilo_ticks;
                // Figure out what color we should be at the current number of neighbors
                get_kilo_color();
            }
            break;
        // If we're not flashing, just get the color for our the current number of neighbors
        case NOT_FLASHING:
            get_kilo_color();
            break;
    }
}

// Check our current movement state and decide what action we need to take depending on it
void check_movement_state()
{
    // Check the flashing state
    switch(flashing_state)
    {
        // If we're flashing, we want to move, so lets check out movement timer
        case FLASHING:
            // If it's roughly been 64 ticks since our last reset
            if((kilo_ticks-motion_timer) == 64)
            {
                set_motion(STOP); // stop motion
            }
            // if it's been rough;y 96 ticks since our last reset
            else if((kilo_ticks-motion_timer) == 96)
            {
                if(num_neighbors != 0) // if we're flashing, but it's not because we're alone
                {
                    set_motion(FORWARD); // move forward
                }   
                motion_timer = kilo_ticks; // reset our motion timer
            }
            break;
        case NOT_FLASHING: // if we're not flashing
            set_motion(STOP); // stop
    }
}

// Function to compare the number of neighbors that each of the nodes in our local network have
void compare_cardinality()
{
    // Now, check out status compared to our neighbors
    uint8_t smallest_size = 17; // default largest size (atm)
    // Smallest size finder algorithm (from online)
    if(neighbors_network_size_map[0] != 0)
    {
        // If the first element is not 0, then use it as the starting value
        smallest_size = neighbors_network_size_map[0];
    }
    // keep comparing the sizes until we find the smallest
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
    // if(num_neighbors < smallest_size) // case for where the SMALLEST in the local network (NO TIES) flashes
    if(num_neighbors <= smallest_size) // case for where the smallest in the local network flashes (INCLUDING TIES - i.e. if node1 and node2 both have 2 neighbors and they're the smallest, then they both flash)
    {
        switch(flashing_state)
        {
            case NOT_FLASHING:
                flashing_state = FLASHING; // Set our flashing state to FLASHING
                flashing_timer = kilo_ticks; // reset our flashing timer
                motion_timer = kilo_ticks; // reset our motion timer
                break;
            case FLASHING: // If we're already flashing, do nothing
                break;
        }
    }
    else
    {
        // If the number of neighbors is NOT the smallest in the local network, don't flash
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
    
    // Set up all of our timers
    start_time = kilo_ticks;
    flashing_timer = kilo_ticks;
    motion_timer = kilo_ticks;
    global_timer = kilo_ticks;
}

// Loopenschnitzel (loop in german [not really])
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

    //---- UNCOMMENT TO FLASH   ----//
    // compare the number of neighbors of each neighbor in the local network
    compare_cardinality();

    // check our flashing state
    check_flashing_state();
    //----                      ----//

    //---- UNCOMMENT TO MOVE    ----//
    if((kilo_ticks-global_timer) > (32*3))
    {
        check_movement_state();
    }

    //---- UNCOMMENT IF NOT FLASHING ----//
    // get_kilo_color();
    //----                           ----//
                                                                                                                                                                                                               
}

// transmit our messages
message_t *message_tx() 
{
    msg.type = NORMAL;
    // Transmit our kilo_uid
    msg.data[0] = (uint8_t) kilo_uid;
    // Transmit the number of neighbors we have
    msg.data[1] = (uint8_t) num_neighbors;
    // Do this thing
    msg.crc = message_crc(&msg);

    return &msg;
}

// receive messages
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
