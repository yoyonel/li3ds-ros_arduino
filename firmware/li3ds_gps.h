#ifndef __LI3DS_GPS__
#define __LI3DS_GPS__


#include "li3ds_time.h"


void gps_setup();
void gps_loop();
//
// const String generateSentenceNMEA();
void updateSentenceNMEA();
char checkSum(String theseChars);


SoftwareSerial gps(10, 4, true); // RX, TX, inverse_logic

char gprmc[96];
// char check;


void gps_setup() {
    // Init Serial communication
    //--------------------------
    // Serial.begin(9600);  // pertube la communication ROSSERIAL
    gps.begin(9600);
    //--------------------------
}

void gps_loop() {
    // update NMEA sentence (contain in global variable: `gprmc`)
    updateSentenceNMEA();
    // Send to NMEA sentence to GPS serial connection
    gps.write(gprmc);
}

void updateSentenceNMEA()
{
    //
    sprintf(gprmc,
            "GPRMC,%2.2u%2.2u%2.2u,A,4901.00,N,200.00,W,0.1,180,01012016,,,S", t4, t3, t2
            );
    //
    const String str(gprmc);
    //
    const char check = checkSum(str);
    String s_check(check, HEX);
    //
    sprintf(gprmc,
            "%c%s%c%s%c", '$', str.c_str(), '*', s_check.c_str(), '\r');
}

/**
 * @brief checkSum
 * @param theseChars
 * @return
 */
char checkSum(String theseChars) {
    char check = 0;
    // iterate over the string, XOR each byte with the total sum:
    for (int c = 0; c < theseChars.length(); c++) {
        check = char(check ^ theseChars.charAt(c));
    }
    // return the result
    return check;
}

// /**
//  * @brief generateSentenceNMEA
//  * @return
//  */
// const String generateSentenceNMEA()
// {
//     //gps.write("$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r");  // OK
    
//     sprintf(gprmc,
//             "GPRMC,%2.2u%2.2u%2.2u,A,4901.00,N,200.00,W,0.1,180,01012016,,,S", t4, t3, t2
//             );

//     String str(gprmc);
//     const char check = checkSum(str);
//     String s_check(check, HEX);
//     // //
//     // str = String('$') + str + String('*') + s_check;
//     // str = String('$') + str + String("*");
//     // str += '\r';
//     sprintf(gprmc,
//             "%c%s%c%s%c", '$', str.c_str(), '*', s_check.c_str(), '\r');

//     // str.toCharArray(gprmc, 96);

//     return str;
// }

#endif // __LI3DS_GPS__