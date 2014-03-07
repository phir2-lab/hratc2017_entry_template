#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad, cos, sin
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseWithCovariance, PoseWithCovarianceStamped
from metal_detector_msgs.msg._Coil import Coil
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# read/write stuff on screen
std = None  

# Robot data
t = Twist()
pose = Pose()
ekfOdom =  Odometry()
radStep = deg2rad(15)
transListener = None

#laser information
laserInfo = LaserScan()

#Inertial Unit 
imuInfo = Imu()

#Metal detector data
leftCoil = Coil()
middleCoil = Coil()
rightCoil = Coil()

# Mine Detection Callback
def receiveCoilSignal(actualCoil):
    if actualCoil.header.frame_id == "left_coil":
        global leftCoil
        leftCoil = actualCoil
    elif actualCoil.header.frame_id == "middle_coil":
        global middleCoil
        middleCoil = actualCoil
    elif actualCoil.header.frame_id == "right_coil":
        global rightCoil
        rightCoil = actualCoil

# Position callback
def receivePosition(actualPose):
    global pose
    pose = actualPose
    showStats()

def receiveEKFOdom(actualOdometry):
    global ekfOdom 
    ekfOdom = actualOdometry
    showStats()

# Laser range-finder callback
def receiveLaser(LaserNow):
    global laserInfo 
    laserInfo = LaserNow

# IMU data callback
def receiveImu(ImuNow):
    global imuInfo 
    imuInfo = ImuNow

# Send mine position to HRATC Framework
def sendMine():
    global transListener, pose

    minePose = Pose()

    # Change middle coil position into world position
    try:    
        (trans,rot) = transListener.lookupTransform('base_footprint', 'middle_coil', rospy.Time(0))
    except:
        return
    cx, cy, cz = trans
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(q)
    minePose.position.x = pose.position.x + cx*cos(yaw) - cy*sin(yaw)
    minePose.position.y = pose.position.y + cx*sin(yaw) + cy*cos(yaw)

    pubMine  = rospy.Publisher('/HRATC_FW/set_mine', Pose)
    pubMine.publish(minePose)
    
# Printing stuff on screen
def showStats():
    if std == None:
        return
    std.clear()
    std.addstr(0,0,"Press Esc to Quit...")
    std.addstr(1,0,"Linear:")
    std.addstr(2, 0, "{} \t {} \t {}".format(t.linear.x,t.linear.y,t.linear.z))
    std.addstr(4,0,"Angular:")
    std.addstr(5, 0, "{} \t {} \t {}".format(t.angular.x,t.angular.y,t.angular.z))
    std.addstr(7,0,"Actual Position:")
    std.addstr(8, 0, "{} \t {} \t {}".format(pose.position.x,pose.position.y,pose.position.z))
    std.addstr(9, 0, "{} \t {} \t {}".format(ekfOdom.pose.pose.position.x,ekfOdom.pose.pose.position.y,ekfOdom.pose.pose.position.z))
    std.addstr(10,0,"Left Coil:")
    std.addstr(11, 3, "Channels: {}".format(leftCoil.channel))
    std.addstr(12, 3, "Zeros: {}".format(leftCoil.zero))
    std.addstr(13,0,"Middle Coil:")
    std.addstr(14, 3, "Channels: {}".format(middleCoil.channel))
    std.addstr(15, 3, "Zeros: {}".format(middleCoil.zero))
    std.addstr(16,0,"Right Coil:")
    std.addstr(17, 3, "Channels: {}".format(rightCoil.channel))
    std.addstr(18, 3, "Zeros: {}".format(rightCoil.zero))

    std.addstr(20, 0, "IMU Quaternion w: {:0.4f} x: {:0.4f} y: {:0.4f} z: {:0.4f} ".format(imuInfo.orientation.w, imuInfo.orientation.x, imuInfo.orientation.y, imuInfo.orientation.z))
    if laserInfo.ranges != []:
        std.addstr(21, 0 , "Laser Readings {} Laser Range Min {:0.4f} Laser Range Max {:0.4f}".format( len(laserInfo.ranges), min(laserInfo.ranges), max(laserInfo.ranges)))

    std.refresh()

# Basic control
def KeyCheck(stdscr):
    stdscr.keypad(True)
    stdscr.nodelay(True)

    k = None
    global std
    std = stdscr

    #publishing topics
    pubVel   = rospy.Publisher('/husky/cmd_vel', Twist)

    # Pan & Tilt Unit - both pan and tilt accept values from -0.5 to 0.5 rad
    pubPTUpan = rospy.Publisher('/ptu_d46/pan_position_controller/command', Float64)
    pubPTUtilt = rospy.Publisher('/ptu_d46/tilt_position_controller/command', Float64)
    PTUpanValue = 0.0
    PTUtiltValue= 0.0

    # Arm
    # Automatic arm sweeping
    pubAutoArmSweep = rospy.Publisher('/arm/sweep', Bool)
    isAutoSweeping = False
    # Manual arm sweeping - the sweep joint accepts values from -0.8 to 0.8
    pubManualArmSweep = rospy.Publisher('/arm/sweep_controller/command', Float64)
    armSweepValue = 0.0
    # Manual arm lifting - the lift joint accepts values from -0.5 to 0.15
    pubManualArmLift  = rospy.Publisher('/arm/lift_controller/command', Float64)
    armLiftValue = 0.0

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

        # Arm movement
        if k == "y":
            isAutoSweeping = not(isAutoSweeping)
            pubAutoArmSweep.publish(Bool(isAutoSweeping)) # toggle automatic arm sweeping
        if k == "u":
            armSweepValue+= 0.1
            if armSweepValue > 0.8:
                armSweepValue = 0.8
            pubManualArmSweep.publish(armSweepValue) 
        if k == "i":
            armSweepValue -= 0.1
            if armSweepValue < -0.8:
                armSweepValue = -0.8
            pubManualArmSweep.publish(armSweepValue)
        if k == "o":
            armLiftValue+= 0.1
            if armLiftValue > 0.15:
                armLiftValue = 0.15
            pubManualArmLift.publish(armLiftValue)
        if k == "p":
            armLiftValue-= 0.1
            if armLiftValue < -0.5:
                armLiftValue = -0.5
            pubManualArmLift.publish(armLiftValue)

        # PTU movement
        if k == "h":
            PTUpanValue+= 0.1
            if PTUpanValue > 0.5:
                PTUpanValue = 0.5
            pubPTUpan.publish(PTUpanValue) 
        if k == "j":
            PTUpanValue -= 0.1
            if PTUpanValue < -0.5:
                PTUpanValue = -0.5
            pubPTUpan.publish(PTUpanValue)
        if k == "k":
            PTUtiltValue+= 0.1
            if PTUtiltValue > 0.5:
                PTUtiltValue = 0.5
            pubPTUtilt.publish(PTUtiltValue)
        if k == "l":
            PTUtiltValue-= 0.1
            if PTUtiltValue < -0.5:
                PTUtiltValue = -0.5
            pubPTUtilt.publish(PTUtiltValue)
 
        # Robot movement
        if k == " ":
            t.linear.x  = 0.0           
            t.angular.z = 0.0
        if k == "KEY_LEFT":
            t.angular.z += radStep
        if k == "KEY_RIGHT":
            t.angular.z -= radStep
        if k == "KEY_UP":
            t.linear.x =  1.0
        if k == "KEY_DOWN":
            t.linear.x = -1.0

        t.angular.z = min(t.angular.z,deg2rad(90))
        t.angular.z = max(t.angular.z,deg2rad(-90))
        pubVel.publish(t)

        showStats()
        time.sleep(0.1)

    stdscr.keypad(False)
    rospy.signal_shutdown("Shutdown Competitor")

# Initialize curses stuff
def StartControl():
    wrapper(KeyCheck)

# ROSPy stuff
def spin():
    rospy.spin()

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('Competitor')

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/HRATC_FW/pose", Pose, receivePosition)
    rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped, receiveEKFOdom)
    rospy.Subscriber("/coils", Coil, receiveCoilSignal)
    rospy.Subscriber("/imu/data", Imu, receiveImu)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)

    # Added a tf listener to check the position of the coils
    transListener = tf.TransformListener()

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()

