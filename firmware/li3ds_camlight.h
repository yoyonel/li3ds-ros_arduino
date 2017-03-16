#ifndef __LI3DS_CAMLIGHT__
#define __LI3DS_CAMLIGHT__

#include <li3ds_configs.h>
#include <li3ds_ros.h>

inline void take_pic();

//
inline void take_pic() { 
	if(flash_state) {                                     
		#ifdef __LED_FLASH__
		analogWrite(FLASH_PIN, MAX_FLASHLEVEL);
		#endif
		delay(STAB_FLASH_DELAY);                           // on attend la stabilisation de l'eclairage à sa valeur max.
	}

	pinMode(CAM_PIN, OUTPUT);
	digitalWrite(CAM_PIN,LOW);

	delay(CAM_WRITE_DELAY);                                     //1ms

	pinMode(CAM_PIN, INPUT);
	digitalWrite(CAM_PIN,LOW);

	currentShot_ms = millis();
	num_pics++;

	if(flash_state) {
		delay(time_acquisition_delay + FLASH_DELAY);	// attente de : temps acquisition de la prise de photo+ delais constant de 10ms avant de 
		                                            	// couper les LED.
		#ifdef __LED_FLASH__
		analogWrite(FLASH_PIN, MIN_FLASHLEVEL);			//Led niveau bas
		#endif

		delay(STAB_FLASH_DELAY);
	}

	#ifdef __DEBUG__
	const long delta = currentShot_ms - prevShot_ms;
	ros_loginfo(
		"ARDUINO - num_pics/currentShot_ms/delta: %d/%d/%d", 
		num_pics, currentShot_ms, delta
	);
	#endif

	prevShot_ms = currentShot_ms;

	return ;
}

#endif