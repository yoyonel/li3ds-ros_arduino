#ifndef __LI3DS_PPS__
#define __LI3DS_PPS__

const byte ppsPin = PPS_PIN;

inline void loop_pps();

inline void loop_pps() {
	// PPS
	digitalWrite(ppsPin, HIGH);
	delay(20);
	digitalWrite(ppsPin, LOW);
}

#endif