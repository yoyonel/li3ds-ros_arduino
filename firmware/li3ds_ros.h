#ifndef __LI3DS_ROS__
#define __LI3DS_ROS__

#include <li3ds_clock.h>
#include <li3ds_states.h>
// #include <li3ds_commands.h>

#include <arduino_msgs/commands.h>
#define NS_FOR_MSG		arduino_msgs

#define NAME_FOR_MSG	commands
#define ARDUINO_SUB_MSG	NS_FOR_MSG::NAME_FOR_MSG
#define SUB_TOPIC_NAME	"/Arduino/commands"

// ros::NodeHandle nh;
#define ROS_MAX_SUBSCRIBERS 1
#define ROS_MAX_PUBLISHERS  0
#define ROS_INPUT_SIZE      150
#define ROS_OUTPUT_SIZE     150
ros::NodeHandle_<ArduinoHardware, ROS_MAX_SUBSCRIBERS,  ROS_MAX_PUBLISHERS, ROS_INPUT_SIZE, ROS_OUTPUT_SIZE> nh;

#define ros_loginfo(...)			\
	sprintf(ros_log, __VA_ARGS__);	\
	nh.loginfo(ros_log);			\
	nh.spinOnce()

extern volatile uint8_t t4;
extern volatile uint8_t t3;
extern volatile uint8_t t2;
extern volatile uint8_t t1;

extern void toggleFLASH();

inline void configure_ros(const unsigned int& baud_rate);

//--------------------------
// Subscriber
//--------------------------
inline void cb_for_sub(const ARDUINO_SUB_MSG& msg);

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
	#endif

	if( flash_state != msg.state_flash ) {
		// update state
		flash_state = msg.state_flash;
		toggleFLASH();
	}
	start_state = msg.state_start;
	pause_state = msg.state_pause;

	#ifdef __DEBUG__
	ros_loginfo(
		"ARDUINO - t2 t3 t4: %2.2u - %2.2u - %2.2u",
		t2, t3, t4
	);

	ros_loginfo(
		"ARDUINO - STATES (before/after update) Flash, Start, Pause: %d/%d - %d/%d - %d/%d",
		old_flash_state, flash_state, 
		old_start_state, start_state, 
		old_pause_state, pause_state
	);
	#endif
}

//
inline void configure_ros(const unsigned int& baud_rate) {
	nh.getHardware()->setBaud(baud_rate);
    nh.initNode();
    nh.subscribe(sub_gps);
}

#endif