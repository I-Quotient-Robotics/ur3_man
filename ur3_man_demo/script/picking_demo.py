#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import signal
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations

from std_msgs.msg import String
from pan_tilt_msg.msg import PanTiltCmd
from robotiq_85_msgs.msg import GripperCmd

running = False
velocity_scaling = 0.15
acceleration_scaling = 0.4

def set_pan_tilt(yaw, pitch):
    pan_tilt_m = PanTiltCmd()
    pan_tilt_m.yaw = yaw
    pan_tilt_m.pitch = pitch
    pan_tilt_m.speed = 10

    pan_tilt_pub.publish(pan_tilt_m)

def set_gripper(id, cmd):
    gripper_cmd = GripperCmd()
    gripper_cmd.force = 50.0
    gripper_cmd.speed = 0.03
    gripper_cmd.stop = False
    gripper_cmd.emergency_release = False
    gripper_cmd.emergency_release_dir = 0

    if cmd=='open':
        gripper_cmd.position = 0.75
    elif cmd=='close':
        gripper_cmd.position = 0.0

    if id=='left':
        left_gripper_pub.publish(gripper_cmd)
    elif id=="right":
        right_gripper_pub.publish(gripper_cmd)

def set_left_joints(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6):
    joint_goal = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    if left_group.go(joint_goal, wait=True):
        print 'ur5 move finished'
    
    return True

def set_right_joints(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6):
    joint_goal = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    if right_group.go(joint_goal, wait=True):
        print 'ur5 move finished'
    
    return True

def set_left_tcp(pos_x, pos_y, pos_z, x, y, z, w):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pos_x
    pose_target.position.y = pos_y
    pose_target.position.z = pos_z
    pose_target.orientation.x = x
    pose_target.orientation.y = y
    pose_target.orientation.z = z
    pose_target.orientation.w = w
    left_group.set_pose_target(pose_target)
    try:
        plan = left_group.plan()
    except:
        print 'ur5 plan error'
    rospy.sleep(2)
    if left_group.execute(plan, wait=True):
        print 'ur5 move finished'

    left_group.stop()

    return True

def set_right_tcp(pos_x, pos_y, pos_z, x, y, z, w):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pos_x
    pose_target.position.y = pos_y
    pose_target.position.z = pos_z
    pose_target.orientation.x = x
    pose_target.orientation.y = y
    pose_target.orientation.z = z
    pose_target.orientation.w = w
    right_group.set_pose_target(pose_target)
    try:
        plan = right_group.plan()
    except:
        print 'ur5 plan error'
    rospy.sleep(2)
    if right_group.execute(plan, wait=True):
        print 'ur5 move finished'

    right_group.stop()

    return True

def signal_handler(signum, frame):
    global running
    running = False

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    left_gripper_pub = rospy.Publisher("/left_arm/gripper/cmd", GripperCmd, queue_size=5)
    right_gripper_pub = rospy.Publisher("/right_arm/gripper/cmd", GripperCmd, queue_size=5)
    pan_tilt_pub = rospy.Publisher("/pan_tilt_driver_node/pan_tilt_cmd", PanTiltCmd, queue_size=5)

    rospy.init_node("picking_demo")

    moveit_commander.roscpp_initialize(sys.argv)
    left_group = moveit_commander.MoveGroupCommander("left_arm")
    right_group = moveit_commander.MoveGroupCommander("right_arm")
    rospy.sleep(3)

    left_group.set_max_velocity_scaling_factor(velocity_scaling)
    left_group.set_max_acceleration_scaling_factor(acceleration_scaling)
    left_group.clear_pose_targets()

    right_group.set_max_velocity_scaling_factor(velocity_scaling)
    right_group.set_max_acceleration_scaling_factor(acceleration_scaling)
    right_group.clear_pose_targets()

    set_pan_tilt(-40, 50)
    rospy.sleep(10)

    set_pan_tilt(40, 50)
    rospy.sleep(10)

    set_pan_tilt(0, 50)
    rospy.sleep(10)

    ################ setup ##################

    rospy.loginfo("left arm ready")
    set_left_joints(2.4478600025177, -0.1979435125934046, 1.255448341369629, -0.6100438276873987, -0.7194855848895472, -0.4847653547870081)
    rospy.sleep(2)

    rospy.loginfo("right arm ready")
    set_right_joints(-2.754603687916891, -1.20032245317568, -1.9832289854632776, -1.7021997610675257, 0.900248646736145, -1.2457035223590296)
    rospy.sleep(2)

    rospy.loginfo('gripper ready')
    set_gripper('left', 'open')
    set_gripper('right', 'open')
    rospy.sleep(2)

    ################ gripper the can ##################

    rospy.loginfo("right arm ready")
    set_right_joints(-3.2452805677997034, -2.1834891478167933, -1.8854621092425745, 1.0803266763687134, 1.3698253631591797, -2.370925251637594)
    rospy.sleep(2)

    rospy.loginfo('gripper ready')
    set_gripper('right', 'open')

    rospy.loginfo("right arm in")
    set_right_joints(-3.3868609110461634, -2.4934757391559046, -0.9213951269732874, 0.20067846775054932, 1.7329376935958862, -2.3607986609088343)
    rospy.sleep(2)

    rospy.loginfo("right gripper close")
    set_gripper('right', 'close')
    rospy.sleep(2)

    rospy.loginfo("right arm out")
    set_right_joints(-2.9826365152942103, -2.6787150541888636, -1.3711264769183558, 0.8347622156143188, 1.331223487854004, -2.3299058119403284)
    rospy.sleep(2)

    rospy.loginfo("right arm ready2")
    set_right_joints(-3.192779843007223, -2.4473503271686, -1.8647945562945765, -0.661229435597555, 0.33263716101646423, -1.3501446882831019)
    rospy.sleep(2)

    ################ open the can ##################

    set_gripper('left', 'open')
    rospy.sleep(2)

    rospy.loginfo("left arm into")
    set_left_joints(2.9292142391204834, -0.12448150316347295, 1.0571107864379883, 0.05367541313171387, -0.3477729002581995, -1.1160300413714808)
    rospy.sleep(2)

    rospy.loginfo("left gripper close")
    set_gripper('left', 'close')
    rospy.sleep(2)

    rospy.loginfo("left arm rotate")
    set_left_joints(2.929274320602417, -0.12451726595033819, 1.057051181793213, 0.05356729030609131, -0.3478205839740198, -2.5494938532458704)
    rospy.sleep(2)

    rospy.loginfo("left arm out")
    set_left_joints(2.564694881439209, -0.1642831007586878, 1.2381205558776855, -0.5677550474749964, -0.6247032324420374, -2.0163305441485804)
    rospy.sleep(2)

    rospy.loginfo("left arm ready2")
    set_left_joints(2.782479763031006, -0.9531443754779261, 1.2835664749145508, -1.6527465025531214, 0.008641000837087631, 1.0365111827850342)
    rospy.sleep(2)

    rospy.loginfo("left gripper open")
    set_gripper('left', 'open')
    rospy.sleep(2)

    rospy.loginfo("left arm ready")
    set_left_joints(2.4478600025177, -0.1979435125934046, 1.255448341369629, -0.6100438276873987, -0.7194855848895472, -0.4847653547870081)
    rospy.sleep(2)

    ################ clean the can ##################

    rospy.loginfo("right arm on trash bin")
    set_right_joints(-3.149198118840353, -2.59755784669985, -1.084930721913473, -1.080444637929098, 0.7451963424682617, -1.5099952856646937)
    rospy.sleep(2)

    rospy.loginfo("right gripper rotate")
    set_right_joints(-3.1491859594928187, -2.597569767628805, -1.0850628058062952, -1.0803611914264124, 0.7451963424682617, 0.7387739419937134)
    rospy.sleep(2)

    rospy.loginfo("right arm on trash bin")
    set_right_joints(-3.149198118840353, -2.59755784669985, -1.084930721913473, -1.080444637929098, 0.7451963424682617, -1.5099952856646937)
    rospy.sleep(2)

    rospy.loginfo("right arm throw can")
    set_right_joints(-3.4105079809771937, -3.2174742857562464, -0.26330691972841436, -1.0046666304217737, 0.7669728398323059, -2.109262768422262)
    rospy.sleep(2)

    rospy.loginfo("right gripper open")
    set_gripper('right', 'open')
    rospy.sleep(2)

    ################ setup ##################

    rospy.loginfo("left arm ready")
    set_left_joints(2.4478600025177, -0.1979435125934046, 1.255448341369629, -0.6100438276873987, -0.7194855848895472, -0.4847653547870081)
    rospy.sleep(1)

    rospy.loginfo("right arm ready")
    set_right_joints(-2.754603687916891, -1.20032245317568, -1.9832289854632776, -1.7021997610675257, 0.900248646736145, -1.2457035223590296)
    rospy.sleep(1)


    # print 'Step 2 Send UR5 to home position'
    # if set_joints(-1.5544, -0.1141, -1.9950, -2.6555, 1.6221, -0.0002):
    #     pass
    # else:
    #     print 'move to home error'
    #     exit(-1)
    # rospy.sleep(1)
    
    print('All finished')
