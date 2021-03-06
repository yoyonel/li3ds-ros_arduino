#include <Arduino.h>
//
#include <SoftwareSerial.h>
//
#include <ros.h>
#include <std_msgs/UInt8.h>
//
// url: https://github.com/Robot-Will/Stino/issues/1
#include <Arduino.h>
//
#include <li3ds_configs.h>
#include <li3ds_ros.h>

#include <li3ds_clock.h>
#include <li3ds_states.h>
#include <li3ds_commands.h>
#include <li3ds_camlight.h>
// #include <li3ds_gps.h>

char ros_log[50];
unsigned int baud_rate = BAUD_RATE;

// -------------------------------------------------------------------------------------------
SoftwareSerial gps(RX_PIN, TX_PIN, true); // RX, TX, inverse_logic
char gprmc[70];
// String str_gprmc;
//
inline void loop_gps();
inline unsigned char checkSum(const String& theseChars);
//
inline void loop_gps() {
	// ------------------------------
	// Construction du message NMEA
	// ------------------------------
	// sprintf(gprmc, "GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");
	// t4 = 23;

	sprintf(gprmc,
            "GPRMC,%2.2u%2.2u%2.2u,A,4901.00,N,200.00,W,0.1,180,01012016,,,S", t4, t3, t2
            );

	const String str_gprmc = String(gprmc);
    const unsigned char check_sum = checkSum(str_gprmc);
	// url: http://forum.arduino.cc/index.php?topic=41826.0
	sprintf(gprmc, "$%s*%2X\r", str_gprmc.c_str(), check_sum);
    //
    // sprintf(gprmc, "%s", str_gprmc.c_str());
	// ------------------------------

	// ------------------------------
	// Envoi du message au GPS
	// ------------------------------
	gps.print(gprmc);
	// ------------------------------

	// Serial.println( String("gprmc: " + String(gprmc) + " send to gps.") );
	#ifdef __DEBUG__
    // ros_loginfo("Test Ros");
	ros_loginfo("GPS - gpmrc: %s", gprmc);
	#endif 
}
/**
 * @brief checkSum
 * @param theseChars
 * @return
 */
inline unsigned char checkSum(const String& theseChars) {
    unsigned char check = 0;
    // iterate over the string, XOR each byte with the total sum:
    for (int c = 0; c < theseChars.length(); c++) {
        check = char(check ^ theseChars.charAt(c));
    }
    // return the result
    return check;
}
// -------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------
const byte ppsPin = PPS_PIN;
//
inline void loop_pps();
//
inline void loop_pps() {
	// PPS
	digitalWrite(ppsPin, HIGH);
	delay(20);
	digitalWrite(ppsPin, LOW);
}
// -------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------
inline void loop_clock();
inline void update_clock();
//
inline void loop_clock() {
	update_clock();

	while (millis() - old_time < 1000) {
		//----------------------
        // ROS
        //----------------------
        // Pendant l'attente (temporisation)
        // On continue de faire tourner "la roue" ROS
        // sinon on risque un "Lost device synchronisation ..."
        // car ROS peut considérer trop long l'appel entre deux nh.spinOnce()
        // avec notre temporisition (tous les 1hz)
        nh.spinOnce();
        delay(ROS_DELAY);
        //----------------------
    }

    old_time = millis();
    
    nh.spinOnce();
}

inline void update_clock() {
    t1++;
    if (t1 > (UPDATE_RATE_PER_SECOND - 1)) {
        t1 = 0;
        t2++;
    }
    if (t2 > 59) {
        t2 = 0;
        t3++;
    }
    if (t3 > 59) {
        t3 = 0;
        t4++;
    }
    if (t4 > 23) t4 = 0;
}
// -------------------------------------------------------------------------------------------

inline void configure_ports() {
	pinMode(ppsPin, OUTPUT);  		// id pin relie au "GPS" vers le VLP (simule le trig)

	pinMode(FLASH_PIN, OUTPUT);		// id pin relie aux LEDs pour le flash

	pinMode(CAM_PIN, INPUT);
  	pinMode(CAM_BOOT_HALT, INPUT);
	
	pinMode(RUN_LED, OUTPUT);
}

void setup() {
	gps.begin(9600);

	configure_ports();

	configure_ros(baud_rate);

	setup_clock();
  	//
  	setup_states();

  	toggleFLASH();
}

void loop() {	
	loop_pps();
	
	loop_gps();

	if( !pause_state && start_state ) {
		take_pic();
	}

  	loop_clock();

  	// Publish states
  	// publish_states();
}