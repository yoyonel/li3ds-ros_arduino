// url: http://answers.ros.org/question/202215/rosserial-arduino-tfbroadcaster-contradicts-using-publisher/?answer=206501#post-id-206501
#define __AVR_ATmega8__

// url: https://github.com/Robot-Will/Stino/issues/1
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
// url: https://github.com/Robot-Will/Stino/issues/148
#include <SoftwareSerial.h>
//
#include <ros.h>
//#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include "li3ds_gps.h"
#include "li3ds_time.h"
//#include "li3ds_camlight.h"
#include "li3ds_leds.h"
#include "li3ds_pps.h"

// Ajout d'un message issu de la centrale INS
// On va utiliser l'Arduino pour r�aliser un pont
// entre la SBG-Ellipse-N et le laser VLP16
// #include <ros_arduino/SbgLogGpsPos.h>
#include <sbg_driver/SbgLogGpsPos.h>

inline void activateTrig();
inline void desactivateTrig();
inline void receiveTrig();


const byte interruptPin = 2; // for digital pin 2
const byte ledPin = 13; // for led pin

// variables will change:
volatile int buttonState = 0;         // variable for reading the pushbutton status

// #define ROS_MAX_SUBSCRIBERS 2
// #define ROS_MAX_PUBLISHERS  2
// #define ROS_INPUT_SIZE      80
// #define ROS_OUTPUT_SIZE     150

#define ROS_MAX_SUBSCRIBERS 1
#define ROS_MAX_PUBLISHERS  1
#define ROS_INPUT_SIZE      150
#define ROS_OUTPUT_SIZE     80


// url: http://answers.ros.org/question/28890/using-rosserial-for-a-atmega168arduino-based-motorcontroller/
// ros::NodeHandle  nh;
//ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> nh;
ros::NodeHandle_<ArduinoHardware, ROS_MAX_SUBSCRIBERS,  ROS_MAX_PUBLISHERS, ROS_INPUT_SIZE, ROS_OUTPUT_SIZE> nh;
// ros::NodeHandle nh;     // standard settings (depend on which Arduino board we have)

//#define BAUDS 9600
#define BAUDS 115200    // ps: faire attention � la vitesse de transfert, ca peut �tre sensible
                        // et provoquer des erreurs/warnings avec ROS::Rosserial
//#define BAUDS 57600

//std_msgs::Int32 msg_arduino_trig;
//ros::Publisher chatter("arduino_trig", &msg_arduino_trig);

std_msgs::Int64 msg_arduino_trig;
ros::Publisher pub_arduino_trig("arduino_trig", &msg_arduino_trig);

std_msgs::Header msg_arduino_trig_timestamp;
ros::Publisher pub_arduino_trig_timestamp("arduino_trig_timestamp", &msg_arduino_trig_timestamp);


//--------------------------
// Subscriber: ''
//--------------------------
//
//--------------------------
/**
 * @brief sub_sbglog_callback
 * @param msg
 */
inline void cb_sbglog_gpspos(const sbg_driver::SbgLogGpsPos& msg);
ros::Subscriber<sbg_driver::SbgLogGpsPos> sub_sbglog_gpspos("/INS/SbgLogGpsPos", &cb_sbglog_gpspos);

inline void cb_sbglog_gpspos(const sbg_driver::SbgLogGpsPos& msg) {
}

inline void cb_arduino(const std_msgs::String& msg);
ros::Subscriber<std_msgs::String> sub_arduino("/Arduino/Cmd", &cb_arduino);

inline void cb_arduino(const std_msgs::String& msg) {

}
//--------------------------

/**
 * @brief setup
 */
void setup() {
    // Temporisation au depart
    // delay(3000);

    //------------------
    // Init ROS
    //------------------
    // url: http://answers.ros.org/question/192356/solved-serial-port-read-returned-short-error-with-arduino-uno-via-bluetootle-with-rosserial/
    nh.getHardware()->setBaud(BAUDS);
    nh.initNode();

    // nh.advertise(pub_arduino_trig);
    // nh.advertise(pub_arduino_trig_timestamp);

    // nh.subscribe(sub_sbglog_gpspos);
    // nh.subscribe(sub_arduino);

    //-------------------
    // Init Arduino PINSls
    //-------------------
    pinMode(ledPin, OUTPUT);  // id pin pour la LED
    //-------------------

    pps_setup();

    gps_setup();

    time_setup();

    //--------------------------
    // CamLight
    //--------------------------
    //camlight_setup(nh);

    //--------------------------
    // LEDS
    //--------------------------
    //led_setup(nh);


    //--------------------------
    // Interruption
    //--------------------------
    //
    //  d'utilisation d'une interruption sur pin dediee
    // pour recuperer le trig electrique envoye au VLP
    // Mais ca ne semble pas fonctionner, peut etre des probleme
    // avec les numeros d'id des pins ... a tester en unitaire
    //
    //  pinMode(2, INPUT);  // id pin relie a id pin 12 pour retroback de trig
    //  // ne semble pas fonctionner avec ROSSERIAL !
    //  attachInterrupt(digitalPinToInterrupt(interruptPin), receiveTrig, RISING);
    //  attachInterrupt(interruptPin, receiveTrig, RISING);
    //  attachInterrupt(interruptPin, receiveTrig, FALLING);
    // initialize the pushbutton pin as an input:
    pinMode(interruptPin, INPUT);
    // Attach an interrupt to the ISR vector
    //attachInterrupt(0, pin_ISR, CHANGE);
}

/**
 * @brief loop: main loop du sketch ARDUINO
 */
void loop() {
    // Calcul du temps et temporisation 1hz
    time_loop(nh);  // on passe le gestionnaire de node ROS

    // A la fin de la seconde, on lance les process
    // - Activation du trig electrique: PPS
    // - camlight_loop: activation du trig pour la prise de capture
    // - Envoi a ROS de l'activation du trig
    // - Desactivation du trig
    // - GPS: Envoi du message NMEA
    
    // TRIGs
    // pour l'instant 2 trigs (cables electriques)
    // different pour le PPS et la CamLight
    // ps: On pourrait tout simplement recabler le fil de la CamLight
    // sur la pin du PPS (12)
    activateTrig();
    // camlight_loop();
    //led_loop();

    // problem avec ROSSERIAL et les pins interrupts d'arduino (pb de synch.)
    // on active le trig cote arduino "a la main"
    receiveTrig();

    delay(20);

    desactivateTrig();

    //delay(500 - 100 - 72);

    gps_loop();
}

/**
 * @brief activateTrig
 */
inline void activateTrig()
{
    digitalWrite(ledPin, HIGH);
    pps_activate_trig();
}

/**
 * @brief desactivateTrig
 */
inline void desactivateTrig()
{
    digitalWrite(ledPin, LOW);
    pps_desactivate_trig();
}

/**
 * @brief receiveTrig
 */
inline void receiveTrig()
{
    // url: https://www.arduino.cc/en/Reference/Micros
    msg_arduino_trig.data = micros();  // get Arduino clock
    // pub_arduino_trig.publish( &msg_arduino_trig );  // send ROS message with this clock

    //
    msg_arduino_trig_timestamp.seq = id_trig;
    msg_arduino_trig_timestamp.stamp = nh.now();
    msg_arduino_trig_timestamp.frame_id = gprmc;
    // pub_arduino_trig_timestamp.publish( &msg_arduino_trig_timestamp );

    //
    nh.spinOnce();
}

/**
 * @brief pin_ISR
 */
void pin_ISR() {
    buttonState = digitalRead(interruptPin);
    //  digitalWrite(ledPin, buttonState);
}

