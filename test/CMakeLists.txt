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


  # Global settings
  set(GLOBAL_OUTPUT_PATH ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(CMAKE_INSTALL_PREFIX ${CATKIN_PACKAGE_BIN_DESTINATION})

  include(ExternalProject)

  MESSAGE(GLOBAL_OUTPUT_PATH: ${GLOBAL_OUTPUT_PATH})
  MESSAGE(CATKIN_DEVEL_PREFIX: ${CATKIN_DEVEL_PREFIX})

  ExternalProject_Add(
    libnmea

    #    GIT_REPOSITORY "https://github.com/jacketizer/libnmea.git"
    GIT_REPOSITORY "https://github.com/yoyonel/libnmea.git"

    GIT_TAG "master"

    UPDATE_COMMAND ""
    PATCH_COMMAND ""

    SOURCE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/libnmea"
    CMAKE_ARGS  -DNMEA_BUILD_SHARED_LIB=ON
          -DNMEA_BUILD_EXAMPLES=OFF
          -DNMEA_UNIT_TESTS=FALSE
          -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    TEST_COMMAND ""
  )

  MESSAGE("CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
#  ${CMAKE_CURRENT_BINARY_DIR}/libnmea-prefix/src/libnmea-build/parsers/
#  SET_TARGET_PROPERTIES(libnmea PROPERTIES INSTALL_RPATH "${CMAKE_CURRENT_BINARY_DIR}/libnmea-prefix/src/libnmea-build/parsers/")
#  SET_TARGET_PROPERTIES(libnmea PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

  MESSAGE(path to lib: ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
  MESSAGE(CATKIN_GLOBAL_INCLUDE_DESTINATION: ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
#    ExternalProject_Add_Step(
#      libnmea CopyToBin
#      COMMAND ${CMAKE_COMMAND} -E copy_directory ${GLOBAL_OUTPUT_PATH}/3rdparty/libnmea/bin ${GLOBAL_OUTPUT_PATH}
#      COMMAND ${CMAKE_COMMAND} -E copy_directory ${GLOBAL_OUTPUT_PATH}/3rdparty/libnmea/lib ${GLOBAL_OUTPUT_PATH}
#      DEPENDEES install
#    )
  ExternalProject_Add_Step(
    libnmea CopyParsersToUsrLib
    COMMAND mkdir -p /usr/lib/nmea
    COMMAND cp -f ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/nmea/libgprmc.so /usr/lib/nmea/.
    COMMAND cp -f ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/nmea/libgpgll.so /usr/lib/nmea/.
    COMMAND cp -f ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/nmea/libgpgga.so /usr/lib/nmea/.
    DEPENDEES build
    DEPENDEES install
  )

  #set(libnmea_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/libnmea/include")
#    set(libnmea_INCLUDE_DIRS "${GLOBAL_OUTPUT_PATH}/include")
  set(libnmea_INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include")
  MESSAGE(libnmea_INCLUDE_DIRS: ${libnmea_INCLUDE_DIRS})

  include_directories(${libnmea_INCLUDE_DIRS})

#    set(libnmea_LIBRARIES "/root/catkin_ws/devel/lib/libnmea.so")
#    set(libnmea_LIBRARIES "/root/catkin_ws/build/devel/lib/libnmea.so")
#    set(libnmea_LIBRARIES "/root/catkin_ws/build/ros_arduino/libnmea-prefix/src/libnmea-build/lib/libnmea.so")
  set(libnmea_LIBRARIES "${CMAKE_SHARED_LIBRARY_PREFIX}nmea${CMAKE_SHARED_LIBRARY_SUFFIX}")

  # Global link directories
#  link_directories(${GLOBAL_OUTPUT_PATH})
  LINK_DIRECTORIES(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
  MESSAGE("CATKIN PACKAGE LIB DESTINATION: ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")

  add_rostest_gtest(
    gtest_ros_arduino_sub
    test/ros_arduino_sub.test
    test/test_arduino_sub.cpp
  )

  target_link_libraries(
    gtest_ros_arduino_sub
    ${libnmea_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  add_dependencies(gtest_ros_arduino_sub libnmea)

#  SET_TARGET_PROPERTIES(
#    gtest_ros_arduino_sub
#    PROPERTIES INSTALL_RPATH "${CMAKE_CURRENT_BINARY_DIR}/libnmea-prefix/src/libnmea-build/parsers/"
#  )
#  SET_TARGET_PROPERTIES(
#    gtest_ros_arduino_sub
#    PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE
#  )
#  set(ENV{NMEA_PARSER_PATH} "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/nmea/")
#  MESSAGE("NMEA_PARSER_PATH: $ENV{NMEA_PARSER_PATH}")

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
