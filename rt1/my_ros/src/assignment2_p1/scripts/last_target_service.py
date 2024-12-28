#!/usr/bin/env python3

import rospy
from assignment2_p1.srv import GetLastTarget, GetLastTargetResponse

last_target = {'x': 0.0, 'y': 0.0}

def handle_last_target(req):
    global last_target
    rospy.loginfo("Returning last target coordinates.")
    return GetLastTargetResponse(x=last_target['x'], y=last_target['y'])

if __name__ == '__main__':
    rospy.init_node('last_target_service')
    service = rospy.Service('/get_last_target', GetLastTarget, handle_last_target)
    rospy.loginfo("Service /get_last_target is ready.")
    rospy.spin()
