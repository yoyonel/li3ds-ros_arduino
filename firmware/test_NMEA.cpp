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
//#include <ros_arduino/gps.h>
#include <sbg_driver/gps.h>

//
#define BAUD_RATE          (57600)
//
#define PPS_PIN				10
#define TX_PIN				4
#define RX_PIN				10
#define FLASH_PIN           11     //sortie pwm eclairage
#define CAM_PIN             9     //sortie commande photo aux cameras (toujours LOW, mais INPUT pour haute impedance et OUTPUT pour mettre a la masse)

#define UPDATE_RATE_PER_SECOND 1

// definitions flash 
#define MAX_FLASHLEVEL        255      // Niveau eclairement max (ie flash).
#define MIN_FLASHLEVEL        5        // Niveau eclairement eco.
#define FLASH_OFFLEVEL        0        // default FLASH ON.
#define STAB_FLASH_DELAY      2
#define FLASH_DELAY          10       // Extinction du flash apres prise de photo.

#define ROS_DELAY   100  // attente de 100ms dans la phase de temporisation de notre boucle temporelle
                        // => ~ 10Hz
// //
// #define BUZZER_PIN      5			//buzzer
// #define BUZZER_ON       255      	// Buzzer on .
// #define BUZZER_OFF      0        	// buzzer off.

// definitions cam
#define CAM_WRITE_DELAY      1                         // delay apres ecriture sur pin de camlight

SoftwareSerial gps(RX_PIN, TX_PIN, true); // RX, TX, inverse_logic

const byte ppsPin = PPS_PIN;
unsigned int baud_rate = BAUD_RATE;

char gprmc[96];
uint8_t gprmc_pos[50];
char ros_log[50];

// String str_gprmc;
volatile uint8_t t4 = 0;
volatile uint8_t t3 = 0;
volatile uint8_t t2 = 0;
volatile uint8_t t1 = 0;
//
unsigned long old_time;

// Cam & pics
int num_pics;                           		// number of pics.
unsigned long 	prevShot_ms;              		// prev time pics in ms.
unsigned long 	currentShot_ms ;          		// current time pics in ms.
unsigned long 	beginWork_ms ;            		// debut chantier en ms
unsigned int 	time_acquisition_delay = 15; 	// Time acquisition of a photo. This value is embedded on SD card

unsigned int time_between_pics = 900;    // Time between two pics in ms.
unsigned int delay_correction = time_acquisition_delay + FLASH_DELAY+STAB_FLASH_DELAY + CAM_WRITE_DELAY;  // correction des temps pour obtenir un intervalle de temps entre photos correct
unsigned int time_between_pics_revised = time_between_pics - delay_correction ;   // temps corrigé

// // states
volatile boolean start_state;
volatile boolean pause_state;
volatile boolean flash_state;

//
inline void loop_pps();
inline void loop_gps();
inline void loop_clock();
//
// inline char checkSum(String theseChars);
inline unsigned char checkSum(const String& theseChars);
inline void update_clock();

inline void toggleFLASH();
inline void toggleSTART();
//
inline void take_pic();
//
inline void configure_ports();
inline void configure_ros();
//


// ros::NodeHandle nh;
#define ROS_MAX_SUBSCRIBERS 1
#define ROS_MAX_PUBLISHERS  0
#define ROS_INPUT_SIZE      150
#define ROS_OUTPUT_SIZE     150
ros::NodeHandle_<ArduinoHardware, ROS_MAX_SUBSCRIBERS,  ROS_MAX_PUBLISHERS, ROS_INPUT_SIZE, ROS_OUTPUT_SIZE> nh;

//--------------------------
// Subscriber
//--------------------------
inline void cb_gps(const sbg_driver::gps& msg);

ros::Subscriber<sbg_driver::gps> sub_gps("/Arduino/gps", &cb_gps);

inline void cb_gps(const sbg_driver::gps& msg) {
	// ------------------------------
	// Récupération des informations depuis le message ROS
	// ROS -> ARDUINO
	// ------------------------------
	// Timer GPS
	t2 = msg.t2_t3_t4[0];
	t3 = msg.t2_t3_t4[1];
	t4 = msg.t2_t3_t4[2];
	//
	// memcpy(gprmc_pos, msg.gprmc_pos, 50);

	if(flash_state != msg.state_flash) {
		toggleFLASH();
	}
	flash_state = msg.state_flash;
	
	if(start_state != msg.state_start) {
		toggleSTART() ;
	}
	start_state = msg.state_start;
	
	if(pause_state != msg.state_pause) {
		if(!pause_state) { 
	        start_state = !start_state;
	        toggleSTART() ;
	    }
    }
	pause_state = msg.state_pause;

	//
	// flash_state: %s
	// flash_state? "True":"False"
	sprintf(ros_log, "t2 t3 t4: %2.2u - %2.2u - %2.2u\tgprmc_pos: %s", 
		t2, t3, t4,
		gprmc_pos
	);
	nh.loginfo(ros_log);	// [OK]
}
//--------------------------

inline void configure_ports() {
	pinMode(ppsPin, OUTPUT);  		// id pin relie au "GPS" vers le VLP (simule le trig)
	pinMode(FLASH_PIN, OUTPUT);		//
	// pinMode(BUZZER_PIN, OUTPUT);
}

inline void configure_ros() {
	nh.getHardware()->setBaud(baud_rate);
    nh.initNode();
    nh.subscribe(sub_gps);
}

void setup() {
	gps.begin(9600);

	configure_ports();

	configure_ros();

    //
  	old_time = 0;

  	//
  	start_state = false;    // stat run, ie take pics
  	pause_state = false;    // stat pause, false
  	flash_state = true;		// flash activate

  	analogWrite(FLASH_PIN, MIN_FLASHLEVEL);     //Led niveau bas
}

void loop() {	
	loop_pps();
	
	loop_gps();

	// analogWrite(BUZZER_PIN, BUZZER_ON);
	if( !pause_state && start_state ) {
		take_pic(); 
	}

  	loop_clock();

  	// analogWrite(BUZZER_PIN, BUZZER_OFF);
  	// Serial.println(String("1 loop executed !")) ;
}

inline void loop_pps() {
	// PPS
	digitalWrite(ppsPin, HIGH);
	delay(20);
	digitalWrite(ppsPin, LOW);
}

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

	Serial.println( String("gprmc: " + String(gprmc) + " send to gps.") );
	nh.loginfo(gprmc);	// [OK]
}

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

inline void toggleFLASH() {
  analogWrite(FLASH_PIN, (flash_state==true ? MIN_FLASHLEVEL : FLASH_OFFLEVEL) );
  // Serial.println ( (flash_state==true?  "FLASH On":"FLASH Off") ); 

  time_between_pics_revised = time_between_pics - (flash_state ? time_acquisition_delay + FLASH_DELAY + STAB_FLASH_DELAY + CAM_WRITE_DELAY : time_acquisition_delay+CAM_WRITE_DELAY);

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

    // sprintf(ros_log, "%s", (start_state ? "START On":"START Off"));
    // nh.loginfo(ros_log);

    return ;
}

inline void take_pic() { 
	if(flash_state) {                                     
		analogWrite(FLASH_PIN, MAX_FLASHLEVEL);
		delay(STAB_FLASH_DELAY);                           // on attend la stabilisation de l'eclairage à sa valeur max.
	}

	// #ifdef USED_BUZZER
	// // start buzzer.
	// analogWrite(BUZZER_PIN, BUZZER_ON);  
	// #endif

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
		analogWrite(FLASH_PIN, MIN_FLASHLEVEL);		//Led niveau bas
	}     

	// #ifdef DEBUG_MSG
	// //  Serial.print("Pics number [") ;
	// //  Serial.print(num_pics) ;
	// //  Serial.print("-") ;
	// //  Serial.print(lastShot_ms) ;
	// //  Serial.println("]") ;
	// #endif  

	// #ifdef USED_BUZZER
	// //stop buzzer
	// anogWrite(BUZZER_PIN, BUZZER_OFF);
	// #endif

	// long delta = currentShot_ms - prevShot_ms ;
	// outputString=String(num_pics)+"-"+String(currentShot_ms)+"-"+String(delta) ; 
	// Serial.println(outputString) ;
	// outputString="";

	prevShot_ms = currentShot_ms;

	return ;
}