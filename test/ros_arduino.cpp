#include <gtest/gtest.h>

#include <ros/ros.h>

#include <thread>

using namespace std;
// using namespace std::chrono;
using namespace std::this_thread;

TEST(ArduinoTestSuite, basicTest){
  EXPECT_TRUE(true);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ArduinoTestNode");
  testing::InitGoogleTest(&argc, argv);

  thread t([]{while(ros::ok()) ros::spin();});

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}