#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float32.h>
#include <cmath>

turtlesim::Pose turtle1_pose, turtle2_pose;

geometry_msgs::Twist turtle1_vel, turtle2_vel;

const float MIN_DISTANCE = 1.5; // Minimum safe distance between turtles
const float MAX_X = 11.0;       // Boundary limits
const float MAX_Y = 11.0;
bool isSafe_b_t = true;
bool turtles_stopped = false;

bool isSafeWithBoundaries(const turtlesim::Pose &pose)
{
    if (pose.x < MIN_DISTANCE || pose.x > MAX_X - MIN_DISTANCE ||
        pose.y < MIN_DISTANCE || pose.y > MAX_Y - MIN_DISTANCE)
    {
        ROS_WARN("Trajectory leads to boundary collision.");
        return false;
    }

    return true;
}

void confirmIsSafe(const std_msgs::Float32::ConstPtr &msg)
{
    bool safe1, safe2, safe3;
    float distance = msg->data;
    // Check if the distance is below the minimum threshold for safety

    if (distance < MIN_DISTANCE)
    {
        ROS_WARN("Trajectory leads to collision with another turtle.");
        safe1 = false; // Collision with another turtle
    }
    else
    {
        safe1 = true; // No collision
    }

    // Check boundary safety for both turtles
    safe2 = isSafeWithBoundaries(turtle1_pose);
    safe3 = isSafeWithBoundaries(turtle2_pose);

    // Update the overall safety status
    isSafe_b_t = safe1 && safe2 && safe3; // All conditions must be safe
}

void publishDistance(const ros::Publisher &pub)
{
    float distance = std::sqrt(std::pow(turtle2_pose.x - turtle1_pose.x, 2) +
                               std::pow(turtle2_pose.y - turtle1_pose.y, 2));

    std_msgs::Float32 msg;
    msg.data = distance;

    pub.publish(msg);
}

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg, const ros::Publisher &pub)
{
    turtle1_pose = *msg;
    publishDistance(pub);
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &msg, const ros::Publisher &pub)
{
    turtle2_pose = *msg;
    publishDistance(pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance2_node");
    ros::NodeHandle nh;
    geometry_msgs::Twist stop_msg; // Message to stop turtles

    stop_msg.linear.x = 0;
    stop_msg.linear.y = 0;
    stop_msg.angular.z = 0;

    ros::Publisher turtle1VelPub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher turtle2VelPub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    ros::Publisher distancePub = nh.advertise<std_msgs::Float32>("/distance", 10);
    ros::Subscriber distanceSub = nh.subscribe<std_msgs::Float32>("/distance", 10, confirmIsSafe);

    ros::Subscriber turtle1Sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10,
                                                               boost::bind(turtle1PoseCallback, _1, boost::ref(distancePub)));
    ros::Subscriber turtle2Sub = nh.subscribe<turtlesim::Pose>("/turtle2/pose", 10,
                                                               boost::bind(turtle2PoseCallback, _1, boost::ref(distancePub)));

    // Main loop
    ros::Rate rate(10); // Run the loop at 10Hz
    while (ros::ok())
    {
        if (!turtles_stopped & !isSafe_b_t) // If the turtles are not safe
        {
            ROS_INFO("Turtles are not safe, stopping them.");
            turtle1VelPub.publish(stop_msg);
            turtle2VelPub.publish(stop_msg);
            turtles_stopped = true;
        }
        else if (turtles_stopped & isSafe_b_t)
        {
            turtles_stopped = false;
            ROS_INFO("Move the turtles to be safe!");
        }

        ros::spinOnce(); // Process callbacks and update the state
        rate.sleep();    // Sleep to maintain the loop rate
    }

    return 0;
}
