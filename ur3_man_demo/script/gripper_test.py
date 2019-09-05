# -*- coding: utf-8 -*-
#!/usr/bin/env python

import sys
import signal
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations

from std_msgs.msg import String
from mir_api import mir_move_goal, get_mir_goal
from moveit_bridge.msg import pose
from pan_tilt_driver.msg import PanTiltCmd
from srt_gripper_driver.msg import SRTGripperCmd

running = False

def set_gripper(cmd, gear):
    gripper_cmd = SRTGripperCmd()
    gripper_cmd.openFlage = False
    gripper_cmd.closeFlage = False
    gripper_cmd.releaseFlage = False
    gripper_cmd.openGears = gear
    gripper_cmd.closeGears = gear

    if cmd==0:
        gripper_cmd.releaseFlage = True
    elif cmd==1:
        gripper_cmd.closeFlage = True
    elif cmd==2:
        gripper_cmd.openFlage = True

    gripper_pub.publish(gripper_cmd)

def signal_handler(signum, frame):
    global running
    running = False

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rospy.init_node("srt_gripper_test")
    gripper_pub = rospy.Publisher("/srt_gripper_driver_node/srt_gripper_cmd", SRTGripperCmd, queue_size=5)
    rospy.sleep(2)

    set_gripper(1, 5)
    rospy.sleep(3)

    set_gripper(2, 5)
    rospy.sleep(3)

    set_gripper(0, 5)

