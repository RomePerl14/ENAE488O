#include "kilolib.h"

// global variables
uint16_t wait_time = 300;

// put your setup code here, will be run once at the beginning
void setup()
{
   
}

// put your main code here, will be run repeatedly
void loop() 
{
  set_color(RGB(0,0,1));
  delay(wait_time);
  set_color(RGB(0,1,1));
  delay(wait_time);
  set_color(RGB(0,1,0));
  delay(wait_time);
  set_color(RGB(1,1,0));
  delay(wait_time);
  set_color(RGB(1,0,0));
  delay(wait_time);
  set_color(RGB(1,0,1));
  delay(wait_time);
}

int main() 
{
  kilo_init();
  kilo_start(setup, loop);

  return 0;
}
