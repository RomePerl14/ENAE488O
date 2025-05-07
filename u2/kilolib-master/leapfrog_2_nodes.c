#include "kilolib.h"

// declare constants
static const uint8_t TOOCLOSE_DISTANCE = 30; // 40 mm
static const uint8_t DESIRED_DISTANCE = 35; // 60 mm

// used to figure out how long we should orbit until we've roughly leaped
int start_time;
int run_time = 30;

// Enumerator to help us know if we are in the front of back of the leap frog
typedef enum
{
    BACK = 0,
    FRONT = 1,
} leap_t;

// Enumerator to help us know if we are moving clockwise or counter clockwise
typedef enum
{
    COUNTER_CLOCKWISE = 1,
    CLOCKWISE = 2,
} move_dir_t;


// declare motion variable type (from orbit-planet.c)
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

// declare state variable type (from orbit-planet.c)
typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
} orbit_state_t;

// declare variables
motion_t cur_motion = STOP; // current motion state
orbit_state_t orbit_state = ORBIT_NORMAL; // current orbit state
uint8_t cur_distance = 0; // current received distance (unmapped)
uint8_t new_message = 0; // indicator if we've received a message or not
distance_measurement_t dist; // distance variable
leap_t current_leap_state; // our current leapfrog state (front of back)
move_dir_t current_direction; // our current rotational direction

uint8_t neighbor_kilo_id; // recevied kilo_uid

uint8_t current_leap_count = 0; // current leap count (local)
uint8_t neighbor_current_leap_count = 0; // current leap count received by our neighbor

message_t msg; // message variable

// function to set new motion (from orbit-planet.c)
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

// function to orbit normally (from orbit-planet.c)
void orbit_normal() {
    if (cur_distance < TOOCLOSE_DISTANCE) {
        orbit_state = ORBIT_TOOCLOSE;
    } else {
        switch(current_direction)
        {
            case COUNTER_CLOCKWISE:
                if (cur_distance < DESIRED_DISTANCE)
                    set_motion(RIGHT);
                else
                    set_motion(LEFT);
                break;
            case CLOCKWISE:
                if (cur_distance < DESIRED_DISTANCE)
                    set_motion(LEFT);
                else
                    set_motion(RIGHT);
                break;
        }
        
    }
}

// function to account for when we are two close to our center (from orbit-planet.c)
void orbit_tooclose() {
    if (cur_distance >= DESIRED_DISTANCE)
        orbit_state = ORBIT_NORMAL;
    else
        set_motion(FORWARD);
}

// setup schtuff
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

    // get the start time
    start_time = kilo_ticks;

    // set an RGB color based on our kilobot
    if(kilo_uid == 1)
    {
        set_color(RGB(1,0,0));
        current_leap_state = FRONT;
        // previous_leap_state = current_leap_state;
    }
    else if(kilo_uid == 2)
    {
        set_color(RGB(0,1,0));
        current_leap_state = BACK;
        // previous_leap_state = current_leap_state;
    }

    // set our starting rotation direction
    current_direction = COUNTER_CLOCKWISE;
}

// loop tings
void loop() {
    // Update distance estimate with every message
    if(current_leap_count > 10)
    {
        current_leap_count = 0;
        neighbor_current_leap_count = 0;
    }

    // if we've received a new message
    if (new_message) {
        new_message = 0;
        cur_distance = estimate_distance(&dist); // estimate distance
        if(current_leap_count < neighbor_current_leap_count) // if our leap count is less than our neighbors, our neighbor has leaped
        {
            current_leap_count = neighbor_current_leap_count; // set our two leap counts equal so we don't cascade
            
            current_leap_state = BACK; // change our state to back
            start_time = kilo_ticks; // reset our start time
        }
    } 
    else if (cur_distance == 0) // skip state machine if no distance measurement available
    {
        return;
    }

    // Orbit state machine
    if(current_leap_state == BACK) // if we're at the back
    {
            switch(orbit_state) 
            {
                case ORBIT_NORMAL:
                    orbit_normal();
                    break;
                case ORBIT_TOOCLOSE:
                    orbit_tooclose();
                    break;
            }
        
        // move for run_time seconds
        if((kilo_ticks - start_time) >= (run_time * 32))
        {
            // after run_time seconds have passed, switch states (assume we've made it)
            current_leap_state = FRONT;
            current_leap_count = (current_leap_count + 1) % 254; // update our leap counter

            // switch our direction
            switch(current_direction)
            {
                case CLOCKWISE:
                    current_direction = COUNTER_CLOCKWISE;
                    break;
                case COUNTER_CLOCKWISE:
                    current_direction = CLOCKWISE;
                    break;
            }
        }
    }
    // if we're at the front, just stop
    else if(current_leap_state == FRONT)
    {
        set_motion(STOP);
    }
                                                                                                                                                                                                               
}

message_t *message_tx() 
{
    // Transmit my ID number
    msg.type = NORMAL;
    msg.data[0] = (uint8_t) kilo_uid; // share UID with the network
    msg.data[1] = (uint8_t) current_leap_count; // message indicating that this kilobot has made it to their target
    
    msg.crc = message_crc(&msg);

    return &msg;
}

void message_rx(message_t *m, distance_measurement_t *d) {
    new_message = 1;
    dist = *d;
    neighbor_kilo_id = m->data[0];
    neighbor_current_leap_count = m->data[1];
}

int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    kilo_start(setup, loop);

    return 0;
}