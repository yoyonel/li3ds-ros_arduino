How to run tests?
https://github.com/catkin/catkin_tools/issues/72

steup/Ros-Test-Example
https://github.com/steup/Ros-Test-Example
https://github.com/steup/Ros-Test-Example/blob/master/src/cars/src/Test.cpp
https://github.com/steup/Ros-Test-Example/blob/master/src/cars/CMakeLists.txt
=> très intéressant, à s'inspirer !

rostest - Minimum Working Example
http://answers.ros.org/question/181708/rostest-minimum-working-example/?answer=181934#post-id-181934

rostests
http://wiki.ros.org/rostest

Unit testing in ROS
https://github.com/hcrlab/wiki/blob/master/software_engineering/unit_testing.md
=> semble fonctionner ! A lire !

Use CMake-enabled libraries in your CMake project
https://coderwall.com/p/y3zzbq/use-cmake-enabled-libraries-in-your-cmake-project

mfreiholz/cmake-example-external-project
https://github.com/mfreiholz/cmake-example-external-project
=> Copié sa méthode, ça a fonctionné avec libnmea (en external project) et le gtest pour ros_arduino !
Hourrey ! :-D

Use gtest in ROS program 
http://ysonggit.github.io/coding/2014/12/19/use-gtest-in-ros-program.html

simonlynen/catkin_wrap_eternal_project.cmake
https://gist.github.com/simonlynen/8060478
configuration d'un projet extérieur non cmake

ethz-asl/orb_slam_2_catkin
https://github.com/ethz-asl/orb_slam_2_catkin/blob/master/CMakeLists.txt
Dépendance externe d'un projet non CMake (plus complexe)


catkin 0.6.19 documentation
Variables
http://docs.ros.org/jade/api/catkin/html/user_guide/variables.html#install-destinations
Peut être très utile à bien connaitre pour s'en sortir plus facilement avec l'écriture des CMakeLists.txt
pour les projets ROS/Catkin


NOTES:
Problème avec la lib libnmea. Il y a du hardcodage de path de génération de librairies dynamiques.
Ca pointe vers un default path système /usr/lib/libnmea/... pour installer les libs .so des parsers.
Du coup à chaque restart du container, ces libs disparaissent !
Faudrait patcher le CMakeLists.txt de la libnmea, c'est très cracra ce qu'ils ont fait (ils le disent dans les commentaires):
/root/project/cmake_test_external_project/3rdparty/libnmea/CMakeLists.txt
	foreach(PARSER_SRC ${PARSERS_SRCS})

		...

	    # Install to where we expect this to be on the system (hard coded).
	    # Otherwise using absolute paths like this is not the CMake way.
	    install(TARGETS ${PARSER_NAME} DESTINATION /usr/lib/nmea/)
	endforeach()
Finalement ce n'est pas simple ... les .so des parsers sont installés via la commande make install dans /usr/lib/nmea et par conséquent accessible par la lib par la suite.
Faudrait voir la notion de RPATH sous CMake: https://cmake.org/Wiki/CMake_RPATH_handling
C'est peut être la solution du "problème"

2017-03-27: Résolution (partielle) du problème ci-dessus
package: 'ros_arduino'
[BUILD]
target: 'test' -> ros_arduino_sub.test
	CMakeLists.txt
	test_arduino_sub.cpp
	- 3rd lib: libnmea
		+ external_project:
ExternalProject_Add(
    libnmea

    #    GIT_REPOSITORY "https://github.com/jacketizer/libnmea.git"
    GIT_REPOSITORY "https://github.com/yoyonel/libnmea.git"

    GIT_TAG "master"

    UPDATE_COMMAND ""
    PATCH_COMMAND ""

    #SOURCE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/libnmea"
    CMAKE_ARGS  -DNMEA_BUILD_SHARED_LIB=ON
          -DNMEA_BUILD_EXAMPLES=OFF
          -DNMEA_UNIT_TESTS=FALSE
          -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    TEST_COMMAND ""
  )
	=> libnmea.so
		|
		--> parsers: libgpgga.so libgpgll.so libhprmc.so
		libs dynamiques
		-> Déploiement/installation vers ./devel/lib/nmea/:
ExternalProject_Add_Step(
    libnmea CopyParsersToUsrLib
    COMMAND mkdir -p /usr/lib/nmea
    COMMAND cp -f ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/nmea/libgprmc.so /usr/lib/nmea/.
    COMMAND cp -f ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/nmea/libgpgll.so /usr/lib/nmea/.
    COMMAND cp -f ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/nmea/libgpgga.so /usr/lib/nmea/.
    DEPENDEES build
    DEPENDEES install
  )

[RUN]
La 3rd lib utilisation une variable d'env. 'NMEA_PARSER_PATH' pour retrouver ces libs dynamiques.
On set cette var env dans le script d'appel du lancement du test.
/root/project/arduino_launch_gtest_sub.sh:
	#!/bin/bash
	export NMEA_PARSER_PATH=/root/catkin_ws/devel/lib/nmea/
	echo "NMEA_PARSER_PATH set to $NMEA_PARSER_PATH"
	rostest ros_arduino ros_arduino_sub.test
