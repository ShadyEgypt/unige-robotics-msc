#include "ros/ros.h"
#include "turtlebot_controller/Velocity.h"
#include <stdlib.h>
#include <stdio.h>

bool velocity_random(turtlebot_controller::Velocity::Request &req,
                     turtlebot_controller::Velocity::Response &res)
{
    res.x = req.min + (rand() / (RAND_MAX / (req.max - req.min)));
    res.z = req.min + (rand() / (RAND_MAX / (req.max - req.min)));
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/velocity", velocity_random);
    ros::spin();
    return 0;
}