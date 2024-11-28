#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>

turtlesim::Pose turtle1_pose, turtle2_pose;

const float MIN_DISTANCE = 0.5; // Minimum safe distance between turtles
const float MAX_X = 11.0;       // Boundary limits
const float MAX_Y = 11.0;

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg)
{
    turtle1_pose = *msg;
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &msg)
{
    turtle2_pose = *msg;
}

bool isSafe(const turtlesim::Pose &pose, const geometry_msgs::Twist &vel, const turtlesim::Pose &other_pose)
{
    bool isSafe_b_t = true;    // Overall safety status
    const float delta_t = 0.1; // Small time step for trajectory discretization
    const int num_steps = 10;  // Number of steps to simulate the trajectory
    float current_x = pose.x;
    float current_y = pose.y;
    float current_theta = pose.theta;
    float future_x = current_x;
    float future_y = current_y;
    ROS_INFO("Current position: x = %.2f, y = %.2f", current_x, current_y);
    ROS_INFO("Received Vel: x = %.2f, y = %.2f", vel.linear.x, vel.linear.y);
    for (int i = 0; i < num_steps; ++i)
    {
        // Incremental time
        float t = delta_t * (i + 1);

        // Calculate intermediate position
        if (std::abs(vel.angular.z) > 1e-6)
        {
            // Motion with angular velocity (circular arc)
            float radius = vel.linear.x / vel.angular.z;
            future_x = pose.x + radius * (sin(current_theta + vel.angular.z * t) - sin(current_theta));
            future_y = pose.y - radius * (cos(current_theta + vel.angular.z * t) - cos(current_theta));
            current_theta += vel.angular.z * t; // Update orientation incrementally
        }
        else if (cos(current_theta) + sin(current_theta) > 1)
        {
            // Straight-line motion influenced by orientation angle
            future_x = current_x + vel.linear.x * cos(current_theta) * delta_t;
            future_y = current_y + vel.linear.y * sin(current_theta) * delta_t;
        }
        else
        {
            // diagonal motion
            future_x = current_x + vel.linear.x * delta_t;
            future_y = current_y + vel.linear.y * delta_t;
        }

        // Boundary collision check
        if (future_x < MIN_DISTANCE || future_x > MAX_X - MIN_DISTANCE ||
            future_y < MIN_DISTANCE || future_y > MAX_Y - MIN_DISTANCE)
        {
            ROS_WARN("Trajectory leads to boundary collision at step %d.", i);
            isSafe_b_t = false;
            break;
        }

        // Collision with the other turtle
        float distance = std::sqrt(std::pow(future_x - other_pose.x, 2) +
                                   std::pow(future_y - other_pose.y, 2));

        if (distance < MIN_DISTANCE)
        {
            ROS_WARN("Trajectory leads to collision with another turtle at step %d.", i);
            isSafe_b_t = false;
            break;
        }

        // Update current position incrementally
        current_x = future_x;
        current_y = future_y;
    }

    // Print the final future position and the distance
    float final_distance = std::sqrt(std::pow(future_x - other_pose.x, 2) +
                                     std::pow(future_y - other_pose.y, 2));
    ROS_INFO("Final future position: x = %.2f, y = %.2f", future_x, future_y);
    ROS_INFO("Final distance to the other turtle: %.2f", final_distance);

    return isSafe_b_t;
}

void proposedVelocityTurtle1Callback(const geometry_msgs::Twist::ConstPtr &msg, ros::Publisher &pub)
{

    if (isSafe(turtle1_pose, *msg, turtle2_pose))
    {
        ROS_INFO("Approved velocity for turtle1.");
        pub.publish(*msg);
    }
    else
    {
        ROS_WARN("Rejected velocity for turtle1.");
    }
}

void proposedVelocityTurtle2Callback(const geometry_msgs::Twist::ConstPtr &msg, ros::Publisher &pub)
{
    if (isSafe(turtle2_pose, *msg, turtle1_pose))
    {
        ROS_INFO("Approved velocity for turtle2.");
        pub.publish(*msg);
    }
    else
    {
        ROS_WARN("Rejected velocity for turtle2.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    ros::Subscriber turtle1Sub = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2Sub = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    ros::Publisher turtle1Pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher turtle2Pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    ros::Subscriber proposedVelTurtle1Sub = nh.subscribe<geometry_msgs::Twist>("/proposed_vel_turtle1", 10,
                                                                               boost::bind(proposedVelocityTurtle1Callback, _1, boost::ref(turtle1Pub)));
    ros::Subscriber proposedVelTurtle2Sub = nh.subscribe<geometry_msgs::Twist>("/proposed_vel_turtle2", 10,
                                                                               boost::bind(proposedVelocityTurtle2Callback, _1, boost::ref(turtle2Pub)));

    ros::spin();
    return 0;
}
