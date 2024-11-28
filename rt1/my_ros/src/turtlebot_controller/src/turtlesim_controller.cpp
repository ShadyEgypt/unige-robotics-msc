#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlebot_controller/Vel.h"
#include "turtlebot_controller/Velocity.h"

double x;

void turtlesimCallback(const turtlesim::Pose::ConstPtr &msg)
{
    ROS_INFO("The position of the turtle is: %f, %f, %f", msg->x, msg->y, msg->theta);
    x = msg->x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesim_listener");
    ros::NodeHandle n;
    ros::Subscriber turtle_sub = n.subscribe("/turtle1/pose", 10, turtlesimCallback);
    ros::Publisher turtle_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::ServiceClient turtle_client = n.serviceClient<turtlesim::Spawn>("/spawn");

    ros::Publisher custom_pub = n.advertise<turtlebot_controller::Vel>("new_vel_topic", 10);
    ros::ServiceClient velocity_client = n.serviceClient<turtlebot_controller::Velocity>("random_velocity");

    turtlesim::Spawn my_spawn;
    my_spawn.request.x = 8.0;
    my_spawn.request.y = 9.0;
    my_spawn.request.theta = 0.0;
    my_spawn.request.name = "new_turtle";

    turtle_client.call(my_spawn);

    ros::Rate loop_rate(10);
    geometry_msgs::Twist my_vel;
    turtlebot_controller::Vel custom_vel;
    turtlebot_controller::Velocity random_vel;

    random_vel.request.min = -2.0;
    random_vel.request.max = 2.0;

    while (ros::ok())
    {
        velocity_client.call(random_vel);
        my_vel.linear.x = random_vel.response.x;
        my_vel.angular.z = random_vel.response.z;
        custom_vel.vel = random_vel.response.x;
        // my_vel.linear.x = 1.0;
        // my_vel.angular.z = 1.0;
        custom_vel.name = "linear velocity";
        custom_vel.vel = 1.0;
        turtle_pub.publish(my_vel);
        custom_pub.publish(custom_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros::spin();
    return 0;
}