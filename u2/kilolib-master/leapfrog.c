/*
    ENAE448O MULTI-ROBOT SWARMS
    Milestone 2 project (U2)
    Group 3
    Romeo Perlstein, Benjamin Tebeest, Oluwatumininu Olanrewaju

    code by:
    Romeo, Ben

    HOLY MOLEY Dr. Otte, I feel like I understand the kilobots on 
    a fundemental level after writing code for them for TWO DAYS
    STRAIGHT!! I understand them, and they understand me...
*/

#include "kilolib.h"
#include <math.h>

// declare constants for orbiting
static const uint8_t TOOCLOSE_DISTANCE = 30; // 40 mm
static const uint8_t DESIRED_DISTANCE = 35; // 60 mm

// Unused Holdover from leapfrog_2_nodes.c
// int start_time = 0;
// int run_time = 6;

// Enumerator to indicate if we are moving clockwise or counterclockwise
typedef enum
{
    COUNTER_CLOCKWISE = 1,
    CLOCKWISE = 2,
} move_dir_t;

// Enumerator to indicate if we are 
//  traveling normally (along the line)
//  Moving into the front position
//  At the front
typedef enum
{
    LEAP_NORMAL,
    LEAP_SHIFT,
    LEAP_FRONT,
} leap_state_t;


// declare motion variable type (from orbit-planet.c.c)
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

// declare state variable type (from orbit-planet.c.c)
typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
} orbit_state_t;

// declare more variables
motion_t cur_motion = STOP; // current motion state
orbit_state_t orbit_state = ORBIT_NORMAL; // current orbit state
uint8_t cur_distance = 0; // current received distance (unconditioned)
uint8_t new_message = 0; // if we got a new message
distance_measurement_t dist; // distance measurement

move_dir_t current_direction; // current direction (CCW, CW)

leap_state_t current_leap_state; // current state of leaping

uint8_t rx_kilo_id; // received kilobot ID

// Neighbor information
uint8_t neighbor1_kilo_id; // Neighbor ID (two kilobots away)
uint8_t neighbor2_kilo_id; // Neighbor ID (right next to us)
uint8_t neighbor1_cur_dist = 0; 
uint8_t neighbor2_cur_dist = 0;
uint8_t neighbor1_cur_leap_location; // neighbor 1 location in the leap order
uint8_t neighbor2_cur_leap_location; // neighbor 2 location in the leap order

uint8_t dist_for_SSS; // neighbor1 -> neighbor2 distance for computing angle using SS triangle

uint8_t orbit_distance = 0; // current orbit distance

uint8_t current_leap_count = 0; // indicates how many times we've leaped
uint8_t neighbor_current_leap_count = 0; // indicates how many times our neighbor has leaped

// LEAPING VARIABLES
uint8_t kilo_leap_order[] = {1,2,3,4,5,6}; // starting configuration (does not need to be in kilo_id order, i.e could be {2,1,3,...})
uint8_t num_kilobots = sizeof(kilo_leap_order); // Number of kilobots in the system
int current_leap_location = -1; // our current leap location

message_t msg; // message variable

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

// orbit around the selected node (from orbit-planet.c.c)
void orbit_normal() {
    if (cur_distance < TOOCLOSE_DISTANCE) {
        orbit_state = ORBIT_TOOCLOSE;
    } else {
        switch(current_direction)
        {
            case COUNTER_CLOCKWISE:
                if (orbit_distance < DESIRED_DISTANCE)
                    set_motion(RIGHT);
                else
                    set_motion(LEFT);
                break;
            case CLOCKWISE:
                if (orbit_distance < DESIRED_DISTANCE)
                    set_motion(LEFT);
                else
                    set_motion(RIGHT);
                break;
        }
        
    }
}

// if we get to close, change up what we're doing (from orbit-planet.c_)
void orbit_tooclose() {
    if (orbit_distance >= DESIRED_DISTANCE)
        orbit_state = ORBIT_NORMAL;
    else
        set_motion(FORWARD);
}

// find the location of each kilobot in the network
void find_current_location()
{
    // Find out current order
    for(uint8_t i=0;i<num_kilobots;i+=1)
    {
        if(kilo_uid == kilo_leap_order[i]) // if we're in the kilo_leap_order (which we should be), find the location
        {
            current_leap_location = i;
        }
    }
}

// find the neighbors of each kilobot
void find_neighbors()
{
    // Get your neighbors
    // This should maintain that we're always keeping track of the right nodes and the distance we are from
    // them, no matter what the current order is of the network
    if(current_leap_location == 0) // at the front of the line
    {
        neighbor1_kilo_id = kilo_leap_order[current_leap_location+1];
        neighbor1_cur_leap_location = current_leap_location + 1;

        neighbor2_kilo_id = kilo_leap_order[current_leap_location+2];
        neighbor2_cur_leap_location = current_leap_location + 2;
    }
    else if (current_leap_location == 1) // one away from the front
    {
        neighbor1_kilo_id = kilo_leap_order[current_leap_location+1];
        neighbor1_cur_leap_location = current_leap_location + 1;

        neighbor2_kilo_id = kilo_leap_order[current_leap_location-1];
        neighbor2_cur_leap_location = current_leap_location - 1;
    }
    else // anywhere else, we don't need to worry about roll over
    {
        neighbor1_kilo_id = kilo_leap_order[current_leap_location-2];
        neighbor2_kilo_id = kilo_leap_order[current_leap_location-1];

        neighbor1_cur_leap_location = current_leap_location - 2;
        neighbor2_cur_leap_location = current_leap_location - 1;
    }
}

// shift the known location of each kilobot
void shift_locations()
{
    // Shift the order of the kilobot network to account for the last node moving to the front of the line
    uint8_t copy_of_kilo_leap_order[num_kilobots]; // make a copy of the current order
    uint8_t end = num_kilobots-1; // the ending location of the kilobot order

    for(uint8_t i=0;i<num_kilobots;i+=1)
    {
        copy_of_kilo_leap_order[i] = kilo_leap_order[i]; // fill the copy
    }

    // Make the first element the last element
    kilo_leap_order[0] = copy_of_kilo_leap_order[end]; // set the new front position to be the back position
    for(uint8_t i=1;i<num_kilobots;i+=1)
    {
        kilo_leap_order[i] = copy_of_kilo_leap_order[i-1]; // shift every other element to the right one index
    }

    // update the kilobots local understanding of node locations
    find_current_location();
}

// find the identifying color of the kilobot base of kilo_uid
void find_kilo_color()
{
    // currently allows for up to 6 kilobots to be color coded, but does not limit overarching number of kilobots
    if(kilo_uid == 0)
    {
        set_color(RGB(0,0,0));
    }
    else if(kilo_uid == 1)
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
    else if(kilo_uid == 4)
    {
        set_color(RGB(1,1,0));
    }
    else if(kilo_uid == 5)
    {
        set_color(RGB(1,0,1));
    }
    else if(kilo_uid == 6)
    {
        set_color(RGB(0,1,1));
    }
    else
    {
        set_color(RGB(1,1,1));
    }
}

// compute the angle B given, sides a, b, c
float compute_SSS()
{
    // Assuming B is the angle we are looking for
    float a = (float) neighbor2_cur_dist;
    float b = (float) neighbor1_cur_dist;
    float c = (float) dist_for_SSS;
    return( acos( ((c*c) + (a*a) - (b*b)) / (2*a*c) ) );
}

// setup schtuff
void setup() 
{   
    // This code is made for N >= 3 kilobots
    if(num_kilobots == 2) // if there are only two kilbots indicated, hold forever and tell the user by flashing red
    {
        while(1)
        {
            set_color(RGB(1,0,0));
            delay(200);
            set_color(RGB(0,0,0));
        }
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
    
    // find out current color
    find_kilo_color();

    // find current order in the kilo_list
    find_current_location();

    
    if(current_leap_location == -1) // if our ID is not in the list of expected IDs
    {
        while(1) // loop forever, and tell the user by flashing white
        {   
            set_color(RGB(1,1,1));
            delay(200);
            set_color(RGB(0,0,0));
        }
    }

    // find our neighbors
    find_neighbors();

    // start counter-clockwise 
    current_direction = COUNTER_CLOCKWISE;

    // starting state for all nodes
    current_leap_state = LEAP_NORMAL;
}

// now loop
void loop() {
    if (cur_distance == 0) // skip state machine if no distance measurement available
    {
        return;
    }
    else if(neighbor1_cur_dist == 0 || neighbor2_cur_dist == 0) // skip state machine if we have not yet received distances from our neighbors
    {
        return;
    }
    if(new_message == 1) // if we got a new message, turn off our LED to indicate that we've received it
    {
        new_message = 0;
        set_color(RGB(0,0,0));
    }

    // If we're at the end of the order, move to the front
    if(current_leap_location == (num_kilobots-1))
    {
        // do travelling state machine
        if(current_leap_state == LEAP_NORMAL)
        {
            if(neighbor1_cur_dist <= neighbor2_cur_dist) // check if we're closer to neighbor 1 than neighbor 2
            {
                // if we are, and neighbor 1 is at the front, switch states
                if((neighbor1_cur_leap_location-1) < 0)
                {
                    current_leap_state = LEAP_SHIFT;
                    orbit_distance = neighbor1_cur_dist;
                }
                // if we are, but neighbor 1 is not at the front, switch our neighbors and continue traveling up the line
                else
                {
                    neighbor2_kilo_id = neighbor1_kilo_id;
                    neighbor2_cur_leap_location = neighbor1_cur_leap_location;
                    neighbor2_cur_dist = neighbor1_cur_dist;

                    neighbor1_kilo_id = kilo_leap_order[neighbor1_cur_leap_location-1];
                    neighbor1_cur_leap_location = neighbor1_cur_leap_location - 1;
                    neighbor1_cur_dist = 100; // set it to max before we receive a message
                }
            }
        }

        // LEAP-STATE STATE MACHINE
        switch(current_leap_state)
        {
            // If we are normal, just orbit the focused node
            case LEAP_NORMAL:
                // Orbit state machine
                switch(orbit_state) 
                {
                    case ORBIT_NORMAL:
                        orbit_normal();
                        break;
                    case ORBIT_TOOCLOSE:
                        orbit_tooclose();
                        break;
                }
                break;
            // If we are in the shifting phase, set our color to white and start 
            // computing the angle to figure out when we're on the other side of the front node
            case LEAP_SHIFT:
                set_color(RGB(1,1,1)); // set the color to indicate the state switch

                // start computing the angle, and if we are smaller than a certain amount (close to 0), switch states
                if(compute_SSS() < 1e-6)
                {
                    current_leap_state = LEAP_FRONT;
                }

                // Orbit state machine
                switch(orbit_state) 
                {
                    case ORBIT_NORMAL:
                        orbit_normal();
                        break;
                    case ORBIT_TOOCLOSE:
                        orbit_tooclose();
                        break;
                }
                break;

            // If we've made it to the front, do some shutdown procedures
            case LEAP_FRONT:
                // Turn off our LED to indicate we've made it to the front
                set_color(RGB(0,0,0));

                // Set motion to STOP
                set_motion(STOP);

                // Shift our local kilo_order list to account for us reaching the front
                shift_locations();
                find_neighbors(); // Find our new neighbors 
                
                // set a 1 second delay just cause
                delay(1000);

                // Update our leap count so that we can inform the network we've leaped successfully
                current_leap_count = (current_leap_count + 1) % 254;

                // Find our color
                find_kilo_color();
                
                // Switch orbit directions now that we're facing the opposite way
                switch(current_direction)
                {
                    case CLOCKWISE:
                        current_direction = COUNTER_CLOCKWISE;
                        break;
                    case COUNTER_CLOCKWISE:
                        current_direction = CLOCKWISE;
                        break;
                }
                break;
        }
    }
    // If we're not at the very end of the line, just stop
    else
    {
        set_motion(STOP);
    }
                                                                                                                                                                                                               
}

message_t *message_tx() 
{
    msg.type = NORMAL;
    // Transmit our kilo_uid
    msg.data[0] = (uint8_t) kilo_uid; // just incase
    // transmit our local leap counter to the network (indcates whether we've succesfully leaped)
    msg.data[1] = (uint8_t) current_leap_count;
    // (ONLY MEANINGFUL WHEN NOT AT THE END OF THE LINE)
    // transmit out distance to our second neighbor (right next to us) so that the moving kilobot 
    // can use it to calculate the angle between n1 and n2 for the SSS law of cosines calculation
    msg.data[2] = (uint8_t) neighbor2_cur_dist;
    
    // Do this thing
    msg.crc = message_crc(&msg);

    return &msg;
}

void message_rx(message_t *m, distance_measurement_t *d) {
    new_message = 1; // indicate we've received a new message
    dist = *d; // get the distance from our sender
    rx_kilo_id = m->data[0]; // get the received kilo_uid
    neighbor_current_leap_count = m->data[1]; // get the leap_count from our neighbor

    // estimate the current distance
    cur_distance = estimate_distance(&dist);

    if(cur_distance == 0) return; // skip below if the cur_distance is 0

    // map the distance to the correct node
    if(rx_kilo_id == neighbor1_kilo_id)
    {
        neighbor1_cur_dist = cur_distance;
    }
    else if(rx_kilo_id == neighbor2_kilo_id)
    {
        neighbor2_cur_dist = cur_distance;
        // distance from n1-n2
        dist_for_SSS = m->data[2]; // (ONLY MEANINGFUL WHEN AT THE END OF THE LINE)
    }

    // If we've gotten a message, flash our color
    find_kilo_color();

    // Check what leap state we are in, and set the node that we're focusing on
    switch(current_leap_state)
    {
        case LEAP_NORMAL:
            // our node of focuse should be the one right next to us
            orbit_distance = neighbor2_cur_dist;
            break;
        case LEAP_SHIFT:
            // if we're maneuvering to the front, our node of focus is n1
            orbit_distance = neighbor1_cur_dist;
            break;
        case LEAP_FRONT:
            break;
    }

    // If we received a leap count from a neighbor that's greater than ours, that means someone made it
    // and we need to update our local status
    if(current_leap_count < neighbor_current_leap_count)
    {
        // set it equal so we don't cause a cascading effect and don't check this again
        current_leap_count = neighbor_current_leap_count;
        
        // set our leap state to normal incase we are next to move
        current_leap_state = LEAP_NORMAL;
        shift_locations(); // update our local understanding of the current order
        find_neighbors(); // find our new neighbors
        return;
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
