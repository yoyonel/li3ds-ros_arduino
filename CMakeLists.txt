cmake_minimum_required(VERSION 2.8.3)

project(ros_arduino)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  # pour les messages de la centrale INS
  message_generation
  std_msgs
)

#######
# pour les messages de la centrale INS
#######
add_message_files(
  DIRECTORY msg
  FILES
  gps.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS message_runtime std_msgs
)
#######

# ###########
# ## Build ##
# ###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware arduino ALL)
rosserial_add_client_target(firmware arduino-upload)
