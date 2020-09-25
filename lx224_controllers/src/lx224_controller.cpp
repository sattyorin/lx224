#include <ros/ros.h>
#include "lx224_controllers/lx224.h"
#include "lx224_controllers/SetSingleServoAngle.h"

lx224 LX224;

bool callback(lx224_controllers::SetSingleServoAngle::Request &req,
                                       lx224_controllers::SetSingleServoAngle::Response &res)
{
	uint8_t pos = req.ang*1000/240;
	if (LX224.SERVO_MOVE_TIME_WRITE(req.id, pos, req.speed))
	{
		return true;
	}
	else
	{
		ROS_ERROR("Failed to writePort");
		return false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lx244_controller");
	
	ros::NodeHandle node_handle_;
	ros::NodeHandle priv_node_handle_;
	ros::ServiceServer client = node_handle_.advertiseService("goal_angle", callback);

	std::string port_name;
	int ctl_rate_hz;
	priv_node_handle_.getParam("port_name",port_name);
	priv_node_handle_.getParam("ctl_rate_hz",ctl_rate_hz);

	ROS_INFO("port open ( %s ) ",port_name.data());
	if (LX224.openPort(port_name.data()) != 1) 
	{
		ROS_ERROR("can't open serial port ( %s )",port_name.data());
		return -1;
	}

	ros::AsyncSpinner spinner(1);
    spinner.start();	

	ros::Rate loop_rate(ctl_rate_hz);

	while(ros::ok())
	{
		// ros::spinOnce();
		loop_rate.sleep();
	}

	LX224.closePort();
	return 0;
}

