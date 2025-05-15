#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif

#define MAX_NEIGHBORS 10  // or however many you expect max

// declare motion variable type
typedef enum {
  STOP,
  FORWARD,
  LEFT,
  RIGHT
} motion_t;

// declare state variable type
typedef enum {
  ORBIT_TOOCLOSE,
  ORBIT_NORMAL,
  STATIONARY 
} orbit_state_t;

// declare variables

typedef struct 
{
  uint8_t cur_pos;
  uint8_t cur_target;
  orbit_state_t orbit_state;
  uint8_t cur_distance;
  uint8_t new_message;
  distance_measurement_t dist;
  
  message_t transmit_msg;
} USERDATA;