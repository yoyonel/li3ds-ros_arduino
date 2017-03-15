#ifndef __LI3DS_CLOCK__
#define __LI3DS_CLOCK__

volatile uint8_t t4 = 0;
volatile uint8_t t3 = 0;
volatile uint8_t t2 = 0;
volatile uint8_t t1 = 0;
//
unsigned long old_time;

inline void setup_clock() {
  	old_time = 0;
}

#endif