#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment2_p1.msg import RobotStatus
from geometry_msgs.msg import Twist

last_target = {'x': 0.0, 'y': 0.0}

def odom_callback(msg):
    global status_pub
    position = msg.pose.pose.position
    velocity = msg.twist.twist

    status_msg = RobotStatus()
    status_msg.x = position.x
    status_msg.y = position.y
    status_msg.vel_x = velocity.linear.x
    status_msg.vel_z = velocity.angular.z
    
    status_pub.publish(status_msg)

def send_goal(x, y):
    global client, last_target
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal)
    last_target['x'] = x
    last_target['y'] = y
    rospy.loginfo(f"Sent goal: x={x}, y={y}")
    client.wait_for_result()
    rospy.loginfo("Goal achieved!")

def cancel_goal():
    global client
    client.cancel_goal()
    rospy.loginfo("Goal canceled!")

if __name__ == '__main__':
    rospy.init_node('action_client_node')
    
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    
    rospy.Subscriber('/odom', Odometry, odom_callback)
    status_pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)
    
    try:
        while not rospy.is_shutdown():
            command = input("Enter 'set' to set goal, 'cancel' to cancel goal, 'exit' to quit: ").strip().lower()
            if command == 'set':
                x = float(input("Enter target x: "))
                y = float(input("Enter target y: "))
                send_goal(x, y)
            elif command == 'cancel':
                cancel_goal()
            elif command == 'exit':
                break
    except rospy.ROSInterruptException:
        pass
