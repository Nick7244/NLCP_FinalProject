#!/usr/bin/env python

import rospy
import trajectory_controller

if __name__ == '__main__':
    
    rospy.init_node('trajectory_tracking_controller', anonymous=True)
    server = trajectory_controller.TrajectoryControllerNode()

    loop_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        server.maintain_position()
        loop_rate.sleep()