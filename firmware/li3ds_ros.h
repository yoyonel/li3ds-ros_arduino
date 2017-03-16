#ifndef __LI3DS_ROS__
#define __LI3DS_ROS__

#include <li3ds_clock.h>
#include <li3ds_states.h>
// #include <li3ds_commands.h>

#include <arduino_msgs/commands.h>
#include <arduino_msgs/states.h>

#define NS_FOR_MSG		arduino_msgs

#define NAME_FOR_MSG	commands
#define ARDUINO_SUB_MSG	NS_FOR_MSG::NAME_FOR_MSG
#define SUB_TOPIC_NAME	"/Arduino/commands"

#define PUB_TOPIC_NAME	"/Arduino/states"
#define ARDUINO_PUB_MSG	NS_FOR_MSG::states

#define USE_CUSTOM_NODEHANDLER
//
#ifdef USE_CUSTOM_NODEHANDLER
#define ROS_MAX_SUBSCRIBERS 1
#define ROS_MAX_PUBLISHERS  1
#define ROS_INPUT_SIZE      150
#define ROS_OUTPUT_SIZE     150
ros::NodeHandle_<ArduinoHardware, ROS_MAX_SUBSCRIBERS,  ROS_MAX_PUBLISHERS, ROS_INPUT_SIZE, ROS_OUTPUT_SIZE> nh;
#else
ros::NodeHandle nh;
#endif

#define ros_loginfo(...)			\
	sprintf(ros_log, __VA_ARGS__);	\
	nh.loginfo(ros_log);			\
	nh.spinOnce()

extern volatile uint8_t t4;
extern volatile uint8_t t3;
extern volatile uint8_t t2;
extern volatile uint8_t t1;

extern unsigned int 	num_pics;

extern void toggleFLASH();
extern void toggleBOOT();

inline void configure_ros(const unsigned int& baud_rate);
//
inline void cb_for_sub(const ARDUINO_SUB_MSG& msg);
//
inline void publish_states();

//--------------------------
// Subscriber
//--------------------------
ros::Subscriber<ARDUINO_SUB_MSG> sub_gps(SUB_TOPIC_NAME, &cb_for_sub);

volatile bool bFirst = true;

inline void cb_for_sub(const ARDUINO_SUB_MSG& msg) {
	// ------------------------------
	// Récupération des informations depuis le message ROS
	// ROS -> ARDUINO
	// ------------------------------
	// Timer GPS
	if(msg.update_clock) {
		t2 = msg.t2_t3_t4[0];
		t3 = msg.t2_t3_t4[1];
		t4 = msg.t2_t3_t4[2];
	}
	
	#ifdef __DEBUG__
	const bool old_start_state = start_state;
	const bool old_flash_state = flash_state;
	const bool old_pause_state = pause_state;
	const bool old_boot_state = boot_state;
	#endif

	if( flash_state != msg.state_flash ) {
		// update state
		flash_state = msg.state_flash;
		toggleFLASH();
	}
	start_state = msg.state_start;
	pause_state = msg.state_pause;
	#
	if( boot_state != msg.state_boot ) {
		boot_state = msg.state_boot;
		toggleBOOT();
	}

	#ifdef __DEBUG__
	ros_loginfo(
		"ARDUINO - t2 t3 t4: %2.2u - %2.2u - %2.2u",
		t2, t3, t4
	);

	ros_loginfo(
		"ARDUINO - STATES (before/after update) Flash, Start, Pause: %d/%d - %d/%d - %d/%d - %d/%d",
		old_flash_state, flash_state, 
		old_start_state, start_state, 
		old_pause_state, pause_state,
		old_boot_state, boot_state
	);
	#endif
}

//--------------------------
// Publisher
//--------------------------
ARDUINO_PUB_MSG msg_states;
ros::Publisher pub_states(PUB_TOPIC_NAME, &msg_states);

inline void publish_states() {
	//
	msg_states.t2_t3_t4[0] = t2;
	msg_states.t2_t3_t4[1] = t3;
	msg_states.t2_t3_t4[2] = t4;
	//
	msg_states.state_start = start_state;
	msg_states.state_pause = pause_state;
	msg_states.state_flash = flash_state;
	msg_states.state_boot = boot_state;
	//
	msg_states.num_trigs_for_pics = num_pics;
	//
	pub_states.publish( &msg_states );
}

//
inline void configure_ros(const unsigned int& baud_rate) {
	nh.getHardware()->setBaud(baud_rate);
    nh.initNode();

    nh.subscribe(sub_gps);
    nh.advertise(pub_states);
}

#endif