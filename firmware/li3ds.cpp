/*
 * LI3DS Arduino sketch 
 * ROS Publisher/Subscriber -> States/Commands
 * GPS, Clock
 */

#include <ros.h>

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
#define WITH_PPS
//
#define WITH_GPS
#define WITH_GPS_SOFTWARESERIAL
#define WITH_GPS_NMEA_GENERATED
#define WITH_GPS_NMEA_ADD_CHECKSUM
// #define WITH_GPS_NMEA_ENDINGS_ANTISLASHN
//
#define WITH_STATES
//

#ifdef WITH_STATES

#define WITH_LED_FLASH
#define WITH_CAMLIGHT
#define WITH_CAMLIGHT_TAKEPIC

volatile boolean state_start;
volatile boolean state_pause;
volatile boolean state_flash;
volatile boolean state_boot;
//
// volatile boolean states_updated;
volatile boolean states_flash_updated, states_boot_updated;

#endif  // Fin de: WITH_STATES

#ifdef WITH_LED_FLASH

#define FLASH_PIN           11    // sortie pwm eclairage

// definitions flash 
#define MAX_FLASHLEVEL        255      // Niveau eclairement max (ie flash).
#define MIN_FLASHLEVEL        5        // Niveau eclairement eco.
#define FLASH_OFFLEVEL        0        // default FLASH ON.
#define STAB_FLASH_DELAY      2
#define FLASH_DELAY           10       // Extinction du flash apres prise de photo.

void toggleFLASH();
#endif  // WITH_LED_FLASH


#ifdef WITH_CAMLIGHT
#define CAM_PIN             8     // sortie commande photo aux cameras (toujours LOW, mais INPUT pour haute impedance et OUTPUT pour mettre a la masse)
//
#define CAM_BOOT_HALT       7
#define CAM_HALT_DELAY      500   // delay de 0,5s pour declencher un "halt", 
                                  // attention en dessous il ne se passe rien ou c'est compris comme un boot,
                                  // au dessus c'est un arret system violent (cas du plantage de l'os embarqué)
#define CAM_BOOT_DELAY      100   // delay de 0,1s pour declencher un boot

#ifdef WITH_CAMLIGHT_TAKEPIC
#define CAM_WRITE_DELAY     1     // delay apres ecriture sur pin de camlight

// Cam & pics
unsigned int    num_pics;                         // number of pics.
unsigned long   prevShot_ms;                      // prev time pics in ms.
unsigned long   currentShot_ms ;                  // current time pics in ms.
unsigned long   beginWork_ms ;                    // debut chantier en ms
const unsigned int  time_acquisition_delay = 15;  // Time acquisition of a photo. This value is embedded on SD card

const unsigned int time_between_pics = 900;    // Time between two pics in ms.
const unsigned int delay_correction = time_acquisition_delay + FLASH_DELAY + STAB_FLASH_DELAY + CAM_WRITE_DELAY;  // correction des temps pour obtenir un intervalle de temps entre photos correct
unsigned int time_between_pics_revised = time_between_pics - delay_correction ;   // temps corrigé

void take_pic();
#endif

#endif  // WITH_CAMLIGHT


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
    states_flash_updated = true;
  }
  state_start = msg.state_start;
  state_pause = msg.state_pause;
  #
  if( state_boot != msg.state_boot ) {
    state_boot = msg.state_boot;
    states_boot_updated = true;
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
  while ( (millis() - old_time) < 1000 ) {
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
    // delay(ROS_DELAY);
    old_time_for_ros = millis();
    while(
      ((millis() - old_time) < 1000) && 
      ((millis() - old_time_for_ros) < ROS_DELAY)
    ) {
    }
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


#ifdef WITH_PPS
#define PPS_PIN       10

void loop_pps();

void loop_pps() {
  // PPS
  digitalWrite(PPS_PIN, HIGH);
  delay(20);
  digitalWrite(PPS_PIN, LOW);
}
#endif // WITH_PPS

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
#ifdef WITH_GPS_NMEA_ENDINGS_ANTISLASHN
  sprintf(gprmc, "$%s*%2X\n\n", str_gprmc.c_str(), check_sum);
#else
  sprintf(gprmc, "$%s*%2X\r", str_gprmc.c_str(), check_sum);
#endif  //  WITH_GPS_NMEA_ENDINGS_SLASHR
#endif  // WITH_GPS_NMEA_ADD_CHECKSUM

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

#ifdef WITH_LED_FLASH
void toggleFLASH() {
  analogWrite(FLASH_PIN, (state_flash ? MIN_FLASHLEVEL : FLASH_OFFLEVEL) );
  // time_between_pics_revised = time_between_pics - (flash_state ? time_acquisition_delay + FLASH_DELAY + STAB_FLASH_DELAY + CAM_WRITE_DELAY : time_acquisition_delay+CAM_WRITE_DELAY);
  // Serial.println ( (flash_state==true?  "FLASH On":"FLASH Off") ); 
  // ros_loginfo("ARDUINO - FLASH %s", (flash_state==true?  "FLASH On":"FLASH Off"));
}
#endif  // WITH_LED_FLASH

#ifdef WITH_CAMLIGHT
void toggleBOOT() {
  if(state_boot) {  // si boot_state on "boot" la CAM
    pinMode(CAM_BOOT_HALT, OUTPUT);
    digitalWrite(CAM_BOOT_HALT, LOW);
    delay(CAM_BOOT_DELAY);  // delay de 0,1s pour declencher un boot
    pinMode(CAM_BOOT_HALT, INPUT);
    digitalWrite(CAM_BOOT_HALT, LOW); 
  }
  else {               // sinon on "halt" la CAM
    pinMode(CAM_BOOT_HALT, OUTPUT);
    digitalWrite(CAM_BOOT_HALT, LOW);
    delay(CAM_HALT_DELAY);          // delay de 0,5s pour declencher un "halt", 
                                    // attention en dessous il ne se passe rien ou c'est compris comme un boot,
                                    // au dessus c'est un arret system violent (cas du plantage de l'os embarqué)
    pinMode(CAM_BOOT_HALT, INPUT);
    digitalWrite(CAM_BOOT_HALT, LOW); 
  }
}

#ifdef WITH_CAMLIGHT_TAKEPIC
void take_pic() { 
#ifdef WITH_LED_FLASH
  if(state_flash) {
    analogWrite(FLASH_PIN, MAX_FLASHLEVEL);
    delay(STAB_FLASH_DELAY);                           // on attend la stabilisation de l'eclairage à sa valeur max.
  }
#endif

  pinMode(CAM_PIN, OUTPUT);
  digitalWrite(CAM_PIN,LOW);

  delay(CAM_WRITE_DELAY);                                     //1ms

  pinMode(CAM_PIN, INPUT);
  digitalWrite(CAM_PIN,LOW);

  currentShot_ms = millis();
  num_pics++;

#ifdef WITH_LED_FLASH
  if(state_flash) {
    delay(time_acquisition_delay + FLASH_DELAY);  // attente de : temps acquisition de la prise de photo+ delais constant de 10ms avant de 
                                                  // couper les LED.
    analogWrite(FLASH_PIN, MIN_FLASHLEVEL);     //Led niveau bas    

    delay(STAB_FLASH_DELAY);
  }
#endif  

  prevShot_ms = currentShot_ms;
}
#endif  //  WITH_CAMLIGHT_TAKEPIC
#endif  // WITH_CAMLIGHT

#ifdef WITH_ROS_PUBLISHER
void update_states_message() {  
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
  //
  states_flash_updated = true;
  states_boot_updated = true;
#endif

#ifdef WITH_CAMLIGHT
  states_msg.num_trigs_for_pics = num_pics;
  states_msg.camlight_shot_ms = currentShot_ms;
#else
  states_msg.num_trigs_for_pics = 0;
  states_msg.camlight_shot_ms = 0;
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
  //
  state_boot  = false;   // etat arret

  states_flash_updated = true;
  states_boot_updated = true;
#endif

#ifdef WITH_LED_FLASH
  pinMode(FLASH_PIN, OUTPUT);   // id pin relie aux LEDs pour le flash
#endif  // WITH_LED_FLASH

#ifdef WITH_CAMLIGHT
  pinMode(CAM_PIN, INPUT);
  pinMode(CAM_BOOT_HALT, INPUT);  
  num_pics = 0;
#endif  // WITH_CAMLIGHT

#ifdef WITH_GPS
  gps.begin(9600);
#endif  // WITH_GPS

#ifdef WITH_PPS
  pinMode(PPS_PIN, OUTPUT);      // id pin relie au "GPS" vers le VLP (simule le trig)
#endif  // WITH_PPS
}

void loop()
{

#ifdef WITH_PPS
  loop_pps();
  
#ifdef WITH_GPS
  loop_gps();
#endif  //  WITH_GPS
#endif  // WITH_PPS

#ifdef WITH_CAMLIGHT_TAKEPIC
    if( !state_pause && state_start ) {
      take_pic();
    }
#endif  // WITH_CAMLIGHT_TAKEPIC

#ifdef WITH_STATES
#ifdef WITH_LED_FLASH
  if( states_flash_updated ) {
    toggleFLASH();
    states_flash_updated = false;
  }
#endif  //  WITH_LED_FLASH

#ifdef WITH_CAMLIGHT
  if( states_boot_updated ) {
    toggleBOOT();
    states_boot_updated = false;
  }
#endif  //  WITH_CAMLIGHT
    //
    // states_updated = false;
#endif  // WITH_STATES

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
