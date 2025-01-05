#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment2_p1.msg import RobotStatus
from geometry_msgs.msg import Twist
from assignment2_p1.srv import GetLastTarget, GetLastTargetResponse

current_feedback = {'x': 0.0, 'y': 0.0, 'status': ""}

def odom_callback(msg):
    global status_pub
    # retrieve the current position and velocity from msg
    position = msg.pose.pose.position
    velocity = msg.twist.twist
    # publish the updated values to the custom msg
    status_msg = RobotStatus()
    status_msg.x = position.x
    status_msg.y = position.y
    status_msg.vel_x = velocity.linear.x
    status_msg.vel_z = velocity.angular.z
    
    status_pub.publish(status_msg)

def send_goal(x, y):
    global client
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal, done_cb=goal_done_callback, feedback_cb=goal_feedback_callback)
    set_last_target(x, y)
    rospy.loginfo(f"Sent goal: x={x}, y={y}")

def goal_done_callback(status, result):
    if status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal achieved successfully!")
    else:
        rospy.loginfo("Goal did not complete successfully.")

def goal_feedback_callback(feedback):
    current_feedback['x'] = feedback.actual_pose.position.x
    current_feedback['y'] = feedback.actual_pose.position.y
    current_feedback['status'] = feedback.stat

def cancel_goal():
    global client
    client.cancel_goal()
    rospy.loginfo("Goal canceled!")

def get_last_target():
    rospy.wait_for_service('/get_last_target')
    try:
        get_last_target_service = rospy.ServiceProxy('/get_last_target', GetLastTarget)
        # No need to pass x and y since we are getting the last target
        response = get_last_target_service(False, 0, 0)
        rospy.loginfo(f"Last target was: x={response.res_x}, y={response.res_y}")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def set_last_target(x, y):
    rospy.wait_for_service('/get_last_target')
    try:
        set_last_target_service = rospy.ServiceProxy('/get_last_target', GetLastTarget)
        response = set_last_target_service(True, x, y)  # Set the last target
        if response.success:
            rospy.loginfo("Last target successfully updated.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('action_client_node')
    
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()
    
    rospy.Subscriber('/odom', Odometry, odom_callback)
    status_pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)
    
    try:
        while not rospy.is_shutdown():
            command = input(
                "Enter a command:\n"
                "  'set' - Set a new goal\n"
                "  'cancel' - Cancel the current goal\n"
                "  'status' - Check the status of the current action\n"
                "  'feedback' - Check current feedback\n"
                "  'last' - Retrieve the last target coordinates\n"
                "  'exit' - Quit the program\n"
                "Your choice: "
            ).strip().lower()

            if command == 'set':
                if client.get_state() == actionlib.GoalStatus.ACTIVE:
                    rospy.loginfo("A goal is currently active, cancel it first!")
                else:
                    x = float(input("Enter target x: "))
                    y = float(input("Enter target y: "))
                    send_goal(x, y)
            elif command == 'cancel':
                cancel_goal()
            elif command == 'status':
                if client.get_state() == actionlib.GoalStatus.ACTIVE:
                    rospy.loginfo("Checking action status...")
                    rospy.loginfo(f"Current Status: {current_feedback['status']}.")
                else:
                    rospy.loginfo("The action is not active")
            elif command == 'feedback':
                if client.get_state() == actionlib.GoalStatus.ACTIVE:
                    rospy.loginfo("Checking action feedback...")
                    rospy.loginfo(f"Current position: x={current_feedback['x']}, y={current_feedback['y']}")
                else:
                    rospy.loginfo("The action is not active")
            elif command == 'last':
                rospy.loginfo("Checking last target...")
                get_last_target()
            elif command == 'exit':
                break
    except rospy.ROSInterruptException:
        pass
