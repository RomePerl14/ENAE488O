#include <math.h>
#include "kilolib.h"
#include "ben_HW2.h"
#include <stdio.h>

// REGISTER_USERDATA(USERDATA);


// declare constants
static const uint8_t NUM_ROBOTS = 6; // For N robots
static const uint8_t TOOCLOSE_DISTANCE = 45; // mm
static const uint8_t DESIRED_DISTANCE = 65; // mm

USERDATA* mydata;

/* Helper function for setting motor speed smoothly (from simulator docs)
*/
void smooth_set_motors(uint8_t ccw, uint8_t cw)
{
  // OCR2A = ccw;  OCR2B = cw;  
#ifdef KILOBOT 
  uint8_t l = 0, r = 0;
  if (ccw && !OCR2A) // we want left motor on, and it's off
    l = 0xff;
  if (cw && !OCR2B)  // we want right motor on, and it's off
    r = 0xff;
  if (l || r)        // at least one motor needs spin-up
    {
      set_motors(l, r);
      delay(15);
    }
#endif
  // spin-up is done, now we set the real value
  set_motors(ccw, cw);
}


void set_motion(motion_t new_motion)
{
  switch(new_motion) {
  case STOP:
    smooth_set_motors(0,0);
    break;
  case FORWARD:
    smooth_set_motors(kilo_straight_left, kilo_straight_right);
    break;
  case LEFT:
    smooth_set_motors(kilo_turn_left, 0); 
    break;
  case RIGHT:
    smooth_set_motors(0, kilo_turn_right); 
    break;
  }
}


void orbit_stationary() {
  set_motion(STOP);
}
     
void orbit_normal()
{
    if (mydata->cur_distance < TOOCLOSE_DISTANCE) {
        // Actively repel
        set_motion(FORWARD); // move forward to escape
        mydata->orbit_state = ORBIT_TOOCLOSE;
    } 
    else if (mydata->cur_distance > DESIRED_DISTANCE + 10) {
        // Curve in toward the target
        set_motion(RIGHT); 
    } 
    else if (mydata->cur_distance < DESIRED_DISTANCE - 10) {
        // Curve outward to increase distance
        set_motion(LEFT); 
    } 
    else {
        // Within acceptable range, orbit tangentially
        set_motion(RIGHT); 
    }
}


void orbit_tooclose() {
  if (mydata->cur_distance >= DESIRED_DISTANCE) {
      mydata->orbit_state = ORBIT_NORMAL;
  } else {
      // Slight turn while backing away to avoid bumping
      set_motion(FORWARD);
  }
}




void orbit() {
  // Orbit state machines
  switch(mydata->orbit_state) {
      case ORBIT_NORMAL:
          orbit_normal();
          break;
      case ORBIT_TOOCLOSE:
          orbit_tooclose();
          break;
      case STATIONARY:
          orbit_stationary();
          break;
        }
}


void message_rx(message_t *m, distance_measurement_t *d) {
  uint8_t sender_pos = m->data[0];
  
  // Only consider senders with a lower cur_pos (earlier in the chain)
  if (sender_pos < mydata->cur_pos) {

      // accept the message only if:
      // - its the first one received in this round, or
      // - the sender is closer than the current one
      uint16_t new_dist = estimate_distance(d);
      
      if (!mydata->new_message || new_dist < mydata->cur_distance) {
          mydata->cur_target = sender_pos;
          mydata->dist = *d;
          mydata->cur_distance = new_dist;
          mydata->new_message = 1;
      }
  }
}


void setup_message(void)
{
  mydata->transmit_msg.type = NORMAL;
  mydata->transmit_msg.data[0] = mydata->cur_pos; // position
  mydata->transmit_msg.data[1] = mydata->cur_target; // current node being targeted
  mydata->transmit_msg.data[2] = mydata->orbit_state; // current state
  

  //finally, calculate a message check sum
  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
}

message_t *message_tx() 
{
  return &mydata->transmit_msg;
}

void loop() {
  // Update distance estimate with every message
  if (mydata->new_message) 
  {
    mydata->new_message = 0;
    mydata->cur_distance = estimate_distance(&mydata->dist);
  } 

  if (mydata->cur_pos < NUM_ROBOTS - 1)
      return;

  orbit();
  
}

void setup()
{
  mydata->cur_pos = kilo_uid & 0xff; // lower byte of kilo id
  if (mydata->cur_pos > 0 )
  mydata->cur_target = mydata->cur_pos - 1;
  else
  mydata->cur_target = mydata->cur_pos - 1;
  mydata->orbit_state = ORBIT_NORMAL;
  mydata->cur_distance = 0;
  mydata->new_message = 0;

  setup_message();

  if (mydata->cur_pos < NUM_ROBOTS - 1)
    set_color(RGB(0,0,0)); // color of the stationary bot
  else
    set_color(RGB(3,0,0)); // color of the moving bot
}





int main() {
    kilo_init();
    kilo_message_rx = message_rx;

    // SET_CALLBACK(botinfo, cb_botinfo);
    
 
    if (kilo_uid < NUM_ROBOTS - 1)
    kilo_message_tx = message_tx;
    
    kilo_start(setup, loop);

    return 0;
}