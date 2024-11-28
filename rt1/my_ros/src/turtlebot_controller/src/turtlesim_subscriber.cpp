#include "ros/ros.h"
#include "turtlesim/Pose.h"

void turtlesimCallback(const turtlesim::Pose::ConstPtr &msg)
{
    ROS_INFO("The position of the turtle is: %f, %f, %f", msg->x, msg->y, msg->theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesim_listener");
    ros::NodeHandle n;
    ros::Subscriber turtle_sub = n.subscribe("/turtle1/pose", 10, turtlesimCallback);
    ros::spin();
    return 0;
}