#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"

ros::Publisher pub;

void turtleCallback(const turtlesim::Pose::ConstPtr &msg)
{
	ROS_INFO("Turtle subscriber@[%f, %f, %f]",
			 msg->x, msg->y, msg->theta);
	geometry_msgs::Twist my_vel;

	if (msg->x > 9.0)
	{
		my_vel.linear.x = 1.0;
		my_vel.angular.z = 1.0;
	}
	else if (msg->x < 1.5)
	{
		my_vel.linear.x = 1.0;
		my_vel.angular.z = -1.0;
	}
	else
	{
		my_vel.linear.x = 1.0;
		my_vel.angular.z = 0.0;
	}
	pub.publish(my_vel);
}

int main(int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS //system
	ros::init(argc, argv, "turtlebot_subscriber");
	ros::NodeHandle nh;
	// Define the publisher and the subscriber to turtle's position
	ros::ServiceClient client2 = nh.serviceClient<turtlesim::Kill>("/kill");
	turtlesim::Kill srv2;
	srv2.request.name = "turtle1";
	client2.waitForExistence();
	client2.call(srv2);

	ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv1;
	srv1.request.name = "rpr_turtle";
	srv1.request.x = 2.0;
	srv1.request.y = 1.0;
	srv1.request.theta = 0.0;
	client1.waitForExistence();
	client1.call(srv1);

	pub = nh.advertise<geometry_msgs::Twist>("rpr_turtle/cmd_vel", 1);
	ros::Subscriber sub = nh.subscribe("rpr_turtle/pose", 1, turtleCallback);
	ros::spin();
	return 0;
}