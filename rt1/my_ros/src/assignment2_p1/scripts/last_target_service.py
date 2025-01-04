#!/usr/bin/env python3

import rospy
from assignment2_p1.srv import GetLastTarget, GetLastTargetResponse

last_target = {'x': 0.0, 'y': 0.0}

def handle_last_target(req):
    global last_target
    if req.set_target:
        last_target['x'] = req.x
        last_target['y'] = req.y
        rospy.loginfo(f"Set last target to x={req.x}, y={req.y}")
        return GetLastTargetResponse(res_x=req.x, res_y=req.y, success=True)
    else:
        rospy.loginfo("Returning last target coordinates.")
        return GetLastTargetResponse(res_x=last_target['x'], res_y=last_target['y'], success=True)

if __name__ == '__main__':
    rospy.init_node('last_target_service')
    service = rospy.Service('/get_last_target', GetLastTarget, handle_last_target)
    rospy.loginfo("Service /get_last_target is ready.")
    rospy.spin()
