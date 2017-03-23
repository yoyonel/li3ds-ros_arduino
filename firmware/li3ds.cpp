/*
 * LI3DS Arduino sketch 
 * ROS Publisher/Subscriber -> States/Commands
 * GPS, Clock
 */

#include <ros.h>


uint8_t encoded_states = 0;


#define MAX_STRING_LENGTH 70

// #define WITH_ROS_LOG             // L'utilisation de logs semblent casser pas mal de choses
                                    // Comme la generation de string gprmc
// #define WITH_ROS_LOG_ON_GPS      // semble poser probleme !
// #define WITH_ROS_LOG_ON_ROS_SUB
//
#define WITH_ROSSERIAL
#define WITH_ROS_PUBLISHER
#define WITH_ROS_SUBSCRIBER
//
#define WITH_CLOCK
// #define WITH_CLOCK_DISCRETIZED
//
#define WITH_GPS
#define WITH_GPS_SOFTWARESERIAL
#define WITH_GPS_NMEA_GENERATED
#define WITH_GPS_NMEA_ADD_CHECKSUM
//
#define WITH_STATES
//
// #define WITH_CAMLIGHT

#ifdef WITH_STATES
volatile boolean state_start;
volatile boolean state_pause;
volatile boolean state_flash;
volatile boolean state_boot;
#endif  // Fin de: WITH_STATES

#ifdef WITH_CLOCK
#define UPDATE_RATE_PER_SECOND 1

#ifdef WITH_ROSSERIAL
#define ROS_DELAY   250
#endif

volatile uint8_t t4 = 0;
volatile uint8_t t3 = 0;
volatile uint8_t t2 = 0;
volatile uint8_t t1 = 0;
//
unsigned long old_time;
#endif  // Fin de: WITH_CLOCK

#ifdef WITH_ROSSERIAL

ros::NodeHandle nh;

#ifdef WITH_ROS_LOG
char ros_log[MAX_STRING_LENGTH];

#define ros_loginfo(...)          \
  sprintf(ros_log, __VA_ARGS__);  \
  nh.loginfo(ros_log);            \
  nh.spinOnce()
#endif  // Fin de: WITH_ROS_LOG

#ifdef WITH_ROS_PUBLISHER
#include <arduino_msgs/states.h>

arduino_msgs::states states_msg;
ros::Publisher pub_states("/Arduino/pub/states", &states_msg);

void update_states_message();
#endif

#ifdef WITH_ROS_SUBSCRIBER
#include <arduino_msgs/commands.h>

void cb_for_sub_cmds(const arduino_msgs::commands& msg) {
  // ------------------------------
  // Récupération des informations depuis le message ROS
  // ROS -> ARDUINO
  // ------------------------------
  
#ifdef WITH_CLOCK
  if(msg.update_clock) {
    t2 = msg.t2_t3_t4[0];
    t3 = msg.t2_t3_t4[1];
    t4 = msg.t2_t3_t4[2];
  }
#endif  // Fin de: WITH_CLOCK

#ifdef WITH_STATES
  if( state_flash != msg.state_flash ) {
    // update state
    state_flash = msg.state_flash;
    // toggleFLASH();
  }
  state_start = msg.state_start;
  state_pause = msg.state_pause;
  #
  if( state_boot != msg.state_boot ) {
    state_boot = msg.state_boot;
    // toggleBOOT();
  }
#endif  // Fin de: WITH_STATES

#ifdef WITH_ROS_LOG_ON_ROS_SUB
  ros_loginfo(
    "ROSSUB -> STATES: %d - %d - %d - %d",
    state_flash, state_start, state_pause, state_boot
  );
#endif  // Fin de: WITH_ROS_LOG_ON_ROS_SUB
}

ros::Subscriber<arduino_msgs::commands> sub_cmds(
  "/Arduino/sub/cmds", 
  &cb_for_sub_cmds
);
#endif  // Fin de: WITH_ROS_SUBSCRIBER

#endif  // Fin de: WITH_ROSSERIAL


#ifdef WITH_CLOCK

void loop_clock();
void update_clock();

void update_clock() {
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

void loop_clock() {
  update_clock();

#ifdef WITH_CLOCK_DISCRETIZED
  while (millis() - old_time < 1000) {
#ifdef WITH_ROSSERIAL
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
#endif  // Fin de: WITH_ROSSERIAL
  }
#else
  delay(1000);
#endif  // Fin de: WITH_CLOTH_DISCRETIZED

  //
  old_time = millis();
}
#endif  // Fin de: WITH_CLOCK


#ifdef WITH_GPS

#ifdef WITH_GPS_SOFTWARESERIAL
#include <SoftwareSerial.h>

#define TX_PIN        4
#define RX_PIN        10

SoftwareSerial gps(RX_PIN, TX_PIN, true); // RX, TX, inverse_logic
#endif

char gprmc[MAX_STRING_LENGTH];
// #define gprmc ros_log

#ifdef WITH_GPS_NMEA_GENERATED
#ifdef WITH_GPS_NMEA_ADD_CHECKSUM
unsigned char checkSum(const String& theseChars);
#endif  // Fin de: WITH_GPS_NMEA_CHECKSUM

#endif  // Fin de: WITH_GPS_NMEA

void loop_gps();

void loop_gps() {
#ifdef WITH_GPS_NMEA_GENERATED
  // ------------------------------
  // Construction du message NMEA
  // ------------------------------
  sprintf(gprmc,
            "GPRMC,%2.2u%2.2u%2.2u,A,4901.00,N,200.00,W,0.1,180,01012016,,,S", t4, t3, t2
            );

#ifdef WITH_GPS_NMEA_ADD_CHECKSUM
  const String str_gprmc = String(gprmc);
    const unsigned char check_sum = checkSum(str_gprmc);
  // url: http://forum.arduino.cc/index.php?topic=41826.0
  sprintf(gprmc, "$%s*%2X\r", str_gprmc.c_str(), check_sum);
#endif  // Fin de: WITH_GPS_NMEA_ADD_CHECKSUM

#else
  sprintf(gprmc, "GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");
#endif  // Fin de: WITH_GPS_NMEA_GENERATED

#ifdef WITH_GPS_SOFTWARESERIAL
  // ------------------------------
  // Envoi du message au GPS
  // ------------------------------
  gps.print(gprmc);
  // ------------------------------ 
#endif

#if defined(WITH_ROS_LOG) && defined(WITH_ROS_LOG_ON_GPS)
  ros_loginfo("GPS - gpmrc: %s", gprmc);
#endif
}

#ifdef WITH_GPS_NMEA_ADD_CHECKSUM
unsigned char checkSum(const String& theseChars) {
    unsigned char check = 0;
    // iterate over the string, XOR each byte with the total sum:
    for (int c = 0; c < theseChars.length(); c++) {
        check = char(check ^ theseChars.charAt(c));
    }
    // return the result
    return check;
}
#endif
#endif  // Fin de: WITH_GPS


#ifdef WITH_ROS_PUBLISHER
void update_states_message() {
  // states_msg.encoded_states = encoded_states;
#ifdef WITH_CLOCK
  states_msg.t2_t3_t4[0] = t2;
  states_msg.t2_t3_t4[1] = t3;
  states_msg.t2_t3_t4[2] = t4;
#else
  states_msg.t2_t3_t4[0] = 0;
  states_msg.t2_t3_t4[1] = 0;
  states_msg.t2_t3_t4[2] = 0;
#endif

#ifdef WITH_STATES
  states_msg.state_start  = state_start;
  states_msg.state_pause  = state_pause;
  states_msg.state_flash  = state_flash;
  states_msg.state_boot   = state_boot;
#else
  states_msg.state_start  = false;
  states_msg.state_pause  = false;
  states_msg.state_flash  = false;
  states_msg.state_boot   = false;
#endif

#ifdef WITH_CAMLIGHT
  states_msg.num_trigs_for_pics = 0;
#else
  states_msg.num_trigs_for_pics = 0;
#endif

#ifdef WITH_GPS
  states_msg.gprmc = gprmc;
#else
  states_msg.gprmc = "";
#endif
}
#endif


void setup()
{
#ifdef WITH_ROSSERIAL
  nh.initNode();

#ifdef WITH_ROS_PUBLISHER
  nh.advertise(pub_states);
#endif

#ifdef WITH_ROS_SUBSCRIBER
  nh.subscribe(sub_cmds);
#endif
#endif  // Fin de: WITH_ROSSERIAL

#ifdef WITH_CLOCK
  old_time = 0;
#endif

#ifdef WITH_STATES
  state_start = false;  // stat run, ie take pics
  state_pause = false;  // stat pause, false
  state_flash = true;   // flash activate
  state_boot  = false;   // etat arret
#endif
}

void loop()
{

#ifdef WITH_GPS
  loop_gps();
#endif

#if defined(WITH_ROSSERIAL) && defined(WITH_ROS_PUBLISHER)
  update_states_message();
  
  pub_states.publish( &states_msg );
  nh.spinOnce();
#endif  // Fin de: WITH_ROSSERIAL && WITH_ROS_PUBLISHER

#ifdef WITH_CLOCK
  loop_clock();
#else 
  delay(1000);
#endif  
}
