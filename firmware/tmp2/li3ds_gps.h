#ifndef __LI3DS_GPS__
#define __LI3DS_GPS__

#include <SoftwareSerial.h>

inline void loop_gps();
inline unsigned char checkSum(const String& theseChars);

char gprmc[96];

SoftwareSerial gps(RX_PIN, TX_PIN, true); // RX, TX, inverse_logic

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
	// ------------------------------

	// ------------------------------
	// Envoi du message au GPS
	// ------------------------------
	gps.print(gprmc);
	// ------------------------------

	// Serial.println( String("gprmc: " + String(gprmc) + " send to gps.") );
	#ifdef __DEBUG__
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
#endif