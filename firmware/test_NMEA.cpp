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

#define BAUD_RATE          (57600)
#define PPS_PIN				10
#define TX_PIN				4
#define RX_PIN				10

#define UPDATE_RATE_PER_SECOND 1

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

//
inline void loop_pps();
inline void loop_gps();
inline void loop_clock();
//
// inline char checkSum(String theseChars);
inline unsigned char checkSum(const String& theseChars);
inline void update_clock();

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
	//
	t2 = msg.t2_t3_t4[0];
	t3 = msg.t2_t3_t4[1];
	t4 = msg.t2_t3_t4[2];
	//
	memcpy(gprmc_pos, msg.gprmc_pos, 50);
	//
	sprintf(ros_log, "t2 t3 t4: %2.2u - %2.2u - %2.2u\tgprmc_pos: %s", 
		t2, t3, t4
		,gprmc_pos
		);
	nh.loginfo(ros_log);	// [OK]
}
//--------------------------

void setup() {
	gps.begin(9600);

	pinMode(ppsPin, OUTPUT);  // id pin relie au "GPS" vers le VLP (simule le trig)

	nh.getHardware()->setBaud(baud_rate);
    nh.initNode();

    nh.subscribe(sub_gps);
    //
	// Serial.begin(baud_rate);
	// while (!Serial) { ; }
 //  	Serial.println("Serial communication ready at "+String(baud_rate)+" baud!") ;


  	old_time = 0;
}

void loop() {	
	loop_pps();
	
	loop_gps();

  	loop_clock();

  	// Serial.println(String("1 loop executed !")) ;
}

inline void loop_pps() {
	// PPS
	digitalWrite(ppsPin, HIGH);
	delay(20);
	digitalWrite(ppsPin, LOW);
}

inline void loop_gps() {
	// sprintf(gprmc, "GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W");
	sprintf(gprmc,
            "GPRMC,%2.2u%2.2u%2.2u,A,4901.00,N,200.00,W,0.1,180,01012016,,,S", t4, t3, t2
            );

	const String str_gprmc = String(gprmc);
	const unsigned char check_sum = checkSum(str_gprmc);
	// url: http://forum.arduino.cc/index.php?topic=41826.0
	sprintf(gprmc, "$%s*%2X\r", str_gprmc.c_str(), check_sum);

	gps.print(gprmc);

	// Serial.println( String("gprmc: " + String(gprmc) + " send to gps.") );
	nh.loginfo(gprmc);	// [OK]
}

inline void loop_clock() {
	update_clock();
	while (millis() - old_time < 1000) {
    }
    old_time = millis();
    //
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
