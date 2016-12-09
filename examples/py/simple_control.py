#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Imu
from metal_detector_msgs.msg._Coil import Coil

# read/write stuff on screen
std = None

radStep = deg2rad(15)
linStep = 0.1
transformer = None
transListener = None

# Robot data
robotTwist = Twist()
localEKFPose = PoseStamped()
globalEKFPose = PoseStamped()

#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

#Inertial Unit 
imuInfo = Imu()

#Metal detector data
coils = Coil()

# Position callback
def receiveEKFOdom(ekfOdom):
    global globalEKFPose 
    globalEKFPose.header = ekfOdom.header
    globalEKFPose.pose = ekfOdom.pose.pose

    global localEKFPose, transListener
    localEKFPose = transListener.transformPose('minefield',globalEKFPose)

    showStats()

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
    global coil
    coil = actualCoil

def getCoilPoseFromTF():
    global transListener

    minePose = PoseStamped()

    # Change left coil position into world position
    try:    
        (trans,rot) = transListener.lookupTransform('minefield', 'left_coil', rospy.Time(0))
    except:
        return
    cx, cy, cz = trans
    minePose.pose.position.x=cx
    minePose.pose.position.y=cy
    return minePose

# Send mine position to HRATC Framework
def sendMine():
    global transListener

    minePose = getCoilPoseFromTF()
    pubMine  = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped)
    pubMine.publish(minePose)

# Printing stuff on screen
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
    std.addstr(8, 0, "Global: {} \t {} \t {}".format(globalEKFPose.pose.position.x, globalEKFPose.pose.position.y, globalEKFPose.pose.position.z))
    std.addstr(9, 0, "Local:  {} \t {} \t {}".format(localEKFPose.pose.position.x, localEKFPose.pose.position.y, localEKFPose.pose.position.z))

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

#        showStats()
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
    rospy.init_node('client')

    transListener = tf.TransformListener()
    transformer = tf.Transformer(True, rospy.Duration(10.0))

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped, receiveEKFOdom)
    rospy.Subscriber("/coils", Coil, receiveCoilSignal)
    rospy.Subscriber("/imu/data", Imu, receiveImu)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.Subscriber("/scan_hokuyo", LaserScan, receiveLaserHokuyo)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()

