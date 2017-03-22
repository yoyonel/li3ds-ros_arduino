#ifndef __LI3DS_ROS_SERVICES__
#define __LI3DS_ROS_SERVICES__

#include <std_srvs/Trigger.h>
using namespace std_srvs;

//--------------------------
// Services
//--------------------------
void cb_srv(const Trigger::Request & req, Trigger::Response & res);
ros::ServiceServer<Trigger::Request, Trigger::Response> server("test_srv", &cb_srv);

void cb_srv(const Trigger::Request & req, Trigger::Response & res) {
	res.success = true;
}

inline void configure_ros_services() {
	// Services
    // nh.advertiseService(server);
}
#endif
