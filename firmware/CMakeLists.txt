cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

# SRCS li3ds_arduino.cpp ${ROS_LIB_DIR}/time.cpp
generate_arduino_firmware(
	arduino
	SRCS li3ds_arduino.cpp
	HDRS li3ds_camlight.h li3ds_gps.h li3ds_leds.h li3ds_pps.h li3ds_time.h PILOTE-PLATEFORME-LI3DS.h
	BOARD uno
	PORT /dev/ttyACM0
)
