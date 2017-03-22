#ifndef __LI3DS_STATES__
#define __LI3DS_STATES__

// states
volatile boolean start_state;
volatile boolean pause_state;
volatile boolean flash_state;
volatile boolean boot_state;

inline void setup_states() {
	start_state = false;	// stat run, ie take pics
  	pause_state = false;	// stat pause, false
  	flash_state = true;		// flash activate
  	boot_state = false;		// etat arret
}

#endif