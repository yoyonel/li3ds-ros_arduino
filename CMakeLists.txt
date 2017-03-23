cmake_minimum_required(VERSION 2.8.3)

project(ros_arduino)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  # # pour les messages de la centrale INS
  # message_generation
  # rosserial_msgs
  # std_msgs
  # message_filters
)

#######
# pour les messages de la centrale INS
#######
# add_message_files(
#   DIRECTORY msg
#   FILES
#   # gps.msg
#   commands.msg
#   states.msg
# )
# # 
# generate_messages(
#         DEPENDENCIES
#         std_msgs
# )
# # 
catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS rosserial_arduino rosserial_client
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

# add_dependencies(${PROJECT_NAME}_ros_lib ${PROJECT_NAME}_generate_message)
add_dependencies(
  ${PROJECT_NAME}_ros_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware arduino ALL)
rosserial_add_client_target(firmware arduino-upload)

# catkin_add_nosetests(test/test_arduino.py)

if(CATKIN_ENABLE_TESTING)
  # these dependencies are only needed for unit testing
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
  )
  
  add_definitions(-std=c++11 -Wall -g)

  add_rostest_gtest(
    gtest_ros_arduino 
    test/ros_arduino.test 
    test/test_arduino.cpp
  )
  target_link_libraries(gtest_ros_arduino ${catkin_LIBRARIES})

  install(
    TARGETS gtest_ros_arduino
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


  include(ExternalProject)

  ExternalProject_Add(
    libnmea

    GIT_REPOSITORY "https://github.com/jacketizer/libnmea.git"
    GIT_TAG "master"
    
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    
    SOURCE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/libnmea"
    CMAKE_ARGS  -DNMEA_BUILD_SHARED_LIB=ON 
                -DNMEA_BUILD_EXAMPLES=OFF
                -DNMEA_UNIT_TESTS=FALSE
                -DCMAKE_INSTALL_PREFIX=${GLOBAL_OUTPUT_PATH}/libnmea
    TEST_COMMAND ""
  )

  # ExternalProject_Add_Step(
  #   libnmea CopyToBin
  #   COMMAND ${CMAKE_COMMAND} -E copy_directory ${GLOBAL_OUTPUT_PATH}/libnmea/bin ${GLOBAL_OUTPUT_PATH}
  #   COMMAND ${CMAKE_COMMAND} -E copy_directory ${GLOBAL_OUTPUT_PATH}/libnmea/lib ${GLOBAL_OUTPUT_PATH}
  #   DEPENDEES install
  # )

  set(libnmea_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/libnmea/include")
  set(libnmea_LIBRARIES "${CMAKE_SHARED_LIBRARY_PREFIX}nmea${CMAKE_SHARED_LIBRARY_SUFFIX}")
  include_directories(${libnmea_INCLUDE_DIRS})

  add_rostest_gtest(
    gtest_ros_arduino_sub
    test/ros_arduino_sub.test
    test/test_arduino_sub.cpp
  )

  # Global link directories
  MESSAGE(GLOBAL_OUTPUT_PATH: ${GLOBAL_OUTPUT_PATH})
  link_directories(${GLOBAL_OUTPUT_PATH})

  target_link_libraries(gtest_ros_arduino_sub 
    ${libnmea_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  add_dependencies(gtest_ros_arduino_sub libnmea)

  install(
    TARGETS gtest_ros_arduino_sub
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
    

  # Ne fonctionne pas ... 
  # -> ne copie pas le programme: test/test_arduino.py
  # pour qu'il soit accessible par la suite par rostest (et ros_arduino_2.test)
  # find_package(catkin REQUIRED COMPONENTS
  #   rospy
  # )
  # catkin_package()
  # install(PROGRAMS 
  #   test/test_arduino.py
  #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  # )
  
  # MESSAGE(TOTO: ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()