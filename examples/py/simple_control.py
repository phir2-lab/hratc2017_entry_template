#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Transform
from sensor_msgs.msg import LaserScan, Imu
from metal_detector_msgs.msg._Coil import Coil
from tf import transformations

# read/write stuff on screen
std = None

radStep = deg2rad(15)
linStep = 0.1
transformer = None
transListener = None

# Robot data
robotTwist = Twist()
robotPose = PoseStamped()
leftCoilPose = PoseStamped()

#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

#Inertial Unit 
imuInfo = Imu()

#Metal detector data
coils = Coil()

######################### AUXILIARY FUNCTIONS ############################

# Get a transformation matrix from a geometry_msgs/Pose
def matrix_from_pose_msg(pose):
    t = transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    r = transformations.quaternion_matrix(q)
    return transformations.concatenate_matrices(t, r)

# Get a geometry_msgs/Pose from a transformation matrix
def pose_msg_from_matrix(transformation):
    msg = Pose()
    msg.position.x = transformation[0][3]
    msg.position.y = transformation[1][3]
    msg.position.z = transformation[2][3]
    q = transformations.quaternion_from_matrix(transformation)
    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]
    return msg

######################### GETTING POSES FROM TF ############################

def updateRobotPose():
    global robotPose

    # This function does not get the true robot pose, but only the pose of 'base_link' in the TF
    # you should replace it by the robot pose resulting from a good localization process 

    robotPose = PoseStamped()
    now = rospy.Time.now()
    # Get left coil position in relation to robot
    try:
        transListener.waitForTransform('minefield', 'base_link', now, rospy.Duration(2.0))    
        (trans,rot) = transListener.lookupTransform('minefield', 'base_link', now)
    except:
        return

    tr2 = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))
    robotPose.pose = pose_msg_from_matrix(tr2)

def updateCoilPoseFromTF():
    global transListener, leftCoilPose

    now = rospy.Time.now()
    # Change left coil position into world position
    try:
        transListener.waitForTransform('minefield', 'left_coil', now, rospy.Duration(3.0)) 
        (trans,rot) = transListener.lookupTransform('minefield', 'left_coil', now)
    except:
        return

    tr = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

    leftCoilPose = PoseStamped()
    leftCoilPose.pose = pose_msg_from_matrix(tr)

def updateCoilPoseManually(referencePose):
    global transListener, leftCoilPose

    now = rospy.Time.now()
    # Get left coil pose in relation to robot
    try:
        transListener.waitForTransform('base_link', 'left_coil', now, rospy.Duration(2.0))    
        (trans,rot) = transListener.lookupTransform('base_link', 'left_coil', now)
    except:
        return

    localCoil_Mat = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

    # Use reference robot pose
    robot_Mat = matrix_from_pose_msg(referencePose)

    # Compute corrected coil pose
    corrected_Mat = np.dot(robot_Mat, localCoil_Mat)

    leftCoilPose = PoseStamped()
    leftCoilPose.pose = pose_msg_from_matrix(corrected_Mat)

# Send mine position to HRATC Framework
def sendMine():
    global transListener

    ## You can get the pose of the metal detector from the TF
    ## But this value may be wrong if the robot pose in TF (that is, base_link) is not accurate
#    updateCoilPoseFromTF()

    ## It is better to compute the coil pose in relation to a corrected robot pose
    updateRobotPose()
    updateCoilPoseManually(robotPose.pose)

    pubMine  = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped)
    pubMine.publish(leftCoilPose)

######################### CALLBACKS ############################

# Laser range-finder callback
def receiveLaser(LaserNow):
    global laserInfo 
    laserInfo = LaserNow

def receiveLaserHokuyo(LaserNow):
    global laserInfoHokuyo 
    laserInfoHokuyo = LaserNow

# IMU data callback
def receiveImu(ImuNow):
    global imuInfo 
    imuInfo = ImuNow

# Mine Detection Callback
def receiveCoilSignal(actualCoil):
    global coils
    coils = actualCoil

    updateRobotPose() 
    updateCoilPoseManually(robotPose.pose)  

######################### CURSES STUFF ############################

# Printing data on screen
def showStats():

    if std == None:
        return

    std.clear()
    std.addstr(0,0,"Press Esc to Quit...")
    std.addstr(1,0,"Linear:")
    std.addstr(2, 0, "{} \t {} \t {}".format(robotTwist.linear.x,robotTwist.linear.y,robotTwist.linear.z))
    std.addstr(4,0,"Angular:")
    std.addstr(5, 0, "{} \t {} \t {}".format(robotTwist.angular.x,robotTwist.angular.y,robotTwist.angular.z))
    std.addstr(7,0,"Robot Position:")
    std.addstr(8, 0, "{} \t {} \t {}".format(robotPose.pose.position.x, robotPose.pose.position.y, robotPose.pose.position.z))
    std.addstr(9,0,"Coil Position:")
    std.addstr(10, 0, "left: {} \t {} \t {}".format(leftCoilPose.pose.position.x, leftCoilPose.pose.position.y, leftCoilPose.pose.position.z))

    std.addstr(18, 0, "Coils readings: l: {} \t r: {}".format(coils.left_coil, coils.right_coil))
    std.addstr(19, 0, "IMU Quaternion w: {:0.4f} x: {:0.4f} y: {:0.4f} z: {:0.4f} ".format(imuInfo.orientation.w, imuInfo.orientation.x, imuInfo.orientation.y, imuInfo.orientation.z))
    if laserInfo.ranges != []:
        std.addstr(20, 0 , "Laser Readings {} Range Min {:0.4f} Range Max {:0.4f}".format( len(laserInfo.ranges), min(laserInfo.ranges), max(laserInfo.ranges)))
    if laserInfoHokuyo.ranges != []:
        std.addstr(21, 0 , "Laser Hokuyo Readings {} Range Min {:0.4f} Range Max {:0.4f}".format( len(laserInfoHokuyo.ranges), min(laserInfoHokuyo.ranges), max(laserInfoHokuyo.ranges)))

    std.refresh()

# Basic control
def KeyCheck(stdscr):
    stdscr.keypad(True)
    stdscr.nodelay(True)

    k = None
    global std
    std = stdscr

    #publishing topics
    pubVel   = rospy.Publisher('/p3at/cmd_vel', Twist)

    # While 'Esc' is not pressed
    while k != chr(27):
        # Check no key
        try:
            k = stdscr.getkey()
        except:
            k = None

        # Set mine position: IRREVERSIBLE ONCE SET
        if k == "x":
            sendMine()

        # Robot movement
        if k == " ":
            robotTwist.linear.x  = 0.0           
            robotTwist.angular.z = 0.0
        if k == "KEY_LEFT":
            robotTwist.angular.z += radStep
        if k == "KEY_RIGHT":
            robotTwist.angular.z -= radStep
        if k == "KEY_UP":
            robotTwist.linear.x +=  linStep
        if k == "KEY_DOWN":
            robotTwist.linear.x -= linStep

        robotTwist.angular.z = min(robotTwist.angular.z,deg2rad(90))
        robotTwist.angular.z = max(robotTwist.angular.z,deg2rad(-90))
        robotTwist.linear.x = min(robotTwist.linear.x,1.0)
        robotTwist.linear.x = max(robotTwist.linear.x,-1.0)
        pubVel.publish(robotTwist)

        showStats()
        time.sleep(0.1)

    stdscr.keypad(False)
    rospy.signal_shutdown("Shutdown Competitor")

######################### MAIN ############################

def spin():
    rospy.spin()
    
def StartControl():
    wrapper(KeyCheck)

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('client')

    transListener = tf.TransformListener()

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/coils", Coil, receiveCoilSignal, queue_size = 1)
    rospy.Subscriber("/imu/data", Imu, receiveImu)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()

