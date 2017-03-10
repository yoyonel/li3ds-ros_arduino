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
char gprmc_tmp[67];
char gprmc[67+1+1+2+1];
// char check;


void gps_setup() {
    // Init Serial communication
    //--------------------------
    // Serial.begin(9600);  // pertube la communication ROSSERIAL
    gps.begin(9600);
    //--------------------------
}

void gps_loop() {
    // // update NMEA sentence (contain in global variable: `gprmc`)
    // updateSentenceNMEA();
    // // Send to NMEA sentence to GPS serial connection
    // gps.write(gprmc);

    // gps.write("$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r");  // OK

    // sprintf(gprmc, "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r"); // OK

    // $
    // 70\r
    // sprintf(gprmc_wo_headers,
    //     "GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");
    // const char check = checkSum(String(gprmc));
    // const String s_check(check, HEX);
    // sprintf(gprmc, "$%s*%2X\r", gprmc_wo_headers, 112); // 112 INT = 70 HEX

    // sprintf(gprmc, "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r"); // 112 INT = 70 HEX
    sprintf(gprmc_tmp, "GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");
    // String str_gprmc = String(gprmc_tmp);
    // char check = checkSum(str_gprmc);
    sprintf(gprmc, "$%s*70\r", gprmc_tmp);
    
    // String str_gprmc = String("");
    // sprintf(gprmc, "$%s*%2X\r", str_gprmc.c_str(), 112);

    // Send to NMEA sentence to GPS serial connection
    gps.write(gprmc);
}

void updateSentenceNMEA()
{    
    // sprintf(gprmc_tmp,
    //         "GPRMC,%2.2u%2.2u%2.2u,A,4901.00,N,200.00,W,0.1,180,01012016,,,S", t4, t3, t2
    //         );
    
    // String str(gprmc_tmp);
    // //
    // char check = checkSum(str);
    // String s_check(check, HEX);
    
    // sprintf(gprmc,
    //         "%c%s%c%s%c", '$', str.c_str(), '*', s_check.c_str(), '\r');
    // sprintf(gprmc, "%s", str.c_str());
    // sprintf(gprmc,
    //         "%c%s%c", '$', gprmc_tmp, '\r');

    // sprintf(gprmc,
    //     "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r");
    sprintf(gprmc,
            "GPRMC,%2.2u%2.2u%2.2u,A,4901.00,N,200.00,W,0.1,180,01012016,,,S", t4, t3, t2
            );
    // sprintf(gprmc, "%s", gprmc_tmp);
    // strncpy(gprmc_tmp, gprmc, 40);
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