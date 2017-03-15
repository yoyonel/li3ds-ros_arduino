#ifndef __LI3DS_COMMANDS__
#define __LI3DS_COMMANDS__

#include <li3ds_configs.h>

// Cam & pics
unsigned int 	num_pics;                       	// number of pics.
unsigned long 	prevShot_ms;              			// prev time pics in ms.
unsigned long 	currentShot_ms ;          			// current time pics in ms.
unsigned long 	beginWork_ms ;            			// debut chantier en ms
const unsigned int 	time_acquisition_delay = 15; 	// Time acquisition of a photo. This value is embedded on SD card

const unsigned int time_between_pics = 900;    // Time between two pics in ms.
const unsigned int delay_correction = time_acquisition_delay + FLASH_DELAY + STAB_FLASH_DELAY + CAM_WRITE_DELAY;  // correction des temps pour obtenir un intervalle de temps entre photos correct
unsigned int time_between_pics_revised = time_between_pics - delay_correction ;   // temps corrigé

inline void toggleFLASH();
inline void toggleSTART();

inline void toggleFLASH() {
#ifdef __LED_FLASH__
  analogWrite(FLASH_PIN, (flash_state==true ? MIN_FLASHLEVEL : FLASH_OFFLEVEL) );
#endif

  time_between_pics_revised = time_between_pics - (flash_state ? time_acquisition_delay + FLASH_DELAY + STAB_FLASH_DELAY + CAM_WRITE_DELAY : time_acquisition_delay+CAM_WRITE_DELAY);

  // Serial.println ( (flash_state==true?  "FLASH On":"FLASH Off") ); 
  ros_loginfo("ARDUINO - FLASH %s", (flash_state==true?  "FLASH On":"FLASH Off"));

  return ;
}

inline void toggleSTART() {
	String msg = String("");

    if(start_state) {
		num_pics = 0;                         // init number of pics.
		prevShot_ms = millis();               // init current time pics in ms.
		beginWork_ms = prevShot_ms ;
		currentShot_ms = 0;
    }
    else {
    	beginWork_ms = 0;

    	//
		// unsigned long duration = millis() - beginWork_ms ;
		// // Serial.println ( duration ) ;
		// int hh = floor( duration/3600000 );
		// int mm = floor( (duration - 3600000*hh ) / 60000 );
		// int ss = floor( (duration - 3600000*hh - 60000*mm) / 1000 ) ;

		// // msg=String("END-")+String(hh)+"HH:"+String(mm)+"MM:"+String(ss)+"SS";
		// sprintf(ros_log, "END-%dHH%dMM%dSS", 
		// 	hh, mm, ss
		// );
		// nh.loginfo(ros_log);
    }

    #ifdef __DEBUG__
    ros_loginfo(ros_log, "%s", (start_state ? "START On":"START Off"));
    #endif

    return ;
}

#endif