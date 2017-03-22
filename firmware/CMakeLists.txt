cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# generate_arduino_firmware(
# 	arduino 
# 	SRCS li3ds_sketch.cpp ${ROS_LIB_DIR}/time.cpp
# 	HDRS li3ds_configs.h li3ds_ros.h li3ds_clock.h li3ds_commands.h
# 	BOARD uno
# 	PORT /dev/ttyACM0
# )

# generate_arduino_firmware(
# 	arduino 
# 	SRCS tmp/simple_li3ds.cpp ${ROS_LIB_DIR}/time.cpp
# 	BOARD uno
# 	PORT /dev/ttyACM0
# )

generate_arduino_firmware(
	arduino 
	SRCS li3ds.cpp ${ROS_LIB_DIR}/time.cpp
	BOARD uno
	PORT /dev/ttyACM0
)