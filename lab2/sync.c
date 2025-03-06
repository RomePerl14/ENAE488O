#include "kilolib.h"


// declare variables
uint8_t new_message = 0;
message_t msgs[32];
uint8_t offsets[32];
uint8_t modulo_clock;
int tick_offset = 0;


// initialize empty message
void setup()
{
    for (int i = 0; i < 32; i++) 
    {
        msgs[i].data[0] = i;
        msgs[i].type = NORMAL;
        msgs[i].crc = message_crc(&msgs[i]);
        offsets[i] = 0;
    }
}

void loop() 
{
    if(new_message == 1)
    {
        new_message = 0;
        modulo_clock = ((kilo_ticks-tick_offset)/4)%32; // check if the modulo_clock ==  0
        if(modulo_clock == 0) // If it is 0, blink
        {
            set_color(RGB(0, 0, 1));
            
            // compute offset average
            int total = 0;
            int sum = 0;
            for(int i = 0; i<32;i+=1)
            {
                total += offsets[i];  // total number of neighbor offsets recorded
                sum += i*offsets[i];  // the total offset amounts
                offsets[i] = 0;      // clear the array for next time
            }
            if(total > 0)
            {
                tick_offset += sum/total;
            }
                
        }
    }
    else
    {
        set_color(RGB(1,0,0)); // if we aren't synched, blink white
    }
    delay(3000);
    set_color(RGB(0, 0, 0));
}   


void message_rx(message_t *msg, distance_measurement_t *d) 
{
    if (modulo_clock > msg->data[0]) 
    {                  //if robot is ahead of neighbor
        if ((modulo_clock - msg->data[0]) < 16)     //by less than half a period
                offsets[modulo_clock-msg->data[0]]++;    // then increment the corresponding offset
    } 
    else 
    {
        if (msg->data[0] - modulo_clock > 16)          //if neighbor is ahead by more than half a period
            offsets[modulo_clock + (32-msg->data[0])]++; // also increment the corresponding offset
    }
    new_message = 1;
}

message_t *message_tx() {
    return &msgs[modulo_clock];
}

int main() 
{
  kilo_init();
  kilo_message_rx = message_rx;
  kilo_message_tx = message_tx;
  kilo_start(setup, loop);
  return 0;
}
