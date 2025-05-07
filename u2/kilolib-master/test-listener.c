#include "kilolib.h"

// declare variables
uint8_t new_message = 0;
message_t rcvd_message;

// update even an odd with message reception
void message_rx(message_t *msg, distance_measurement_t *dist) 
{
    rcvd_message = *msg;
    new_message = 1;
}

void setup() 
{ 

}


void loop() 
{
    // Blink led different colors if message recieved
    if (new_message == 1) 
    {
        new_message = 0;
        set_color(RGB(1,0,1)); // blink magenta
        delay(10);
        set_color(RGB(0,0,0)); // turn off

    }
}


int main() 
{
  kilo_init();
  kilo_message_rx = message_rx;
  kilo_start(setup, loop);
  return 0;
}
