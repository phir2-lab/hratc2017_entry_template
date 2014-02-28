#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2
import numpy as np
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose
from metal_detector_msgs.msg._Coil import Coil
from numpy import deg2rad
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan, Imu
from std_msgs.msg import Bool, Float64

# read/write stuff on screen
std = None  

# Robot data
t = Twist()
pose = Pose()
radStep = deg2rad(15)

#laser information
laserInfo = LaserScan()

#Inertial Unit 
imuInfo = Imu()

#Metal detector data
coil = Coil()

# Camera data 
size = 800, 800, 3
leftCamInfo = CameraInfo()
rightCamInfo = CameraInfo()
leftCamFrame = CompressedImage()
rightCamFrame = CompressedImage()
cv2.namedWindow("Left window", 1)
cv2.namedWindow("Right window",1)


# Mine Detection Callback
def receiveMineDetection(actualCoil):
    global coil
    coil = actualCoil

# Position callback
def receivePosition(actualPose):
    global pose
    pose = actualPose
    showStats()

# Camera basic information callbacks
def receiveRightCameraInfo(rightInfNow):
    global rightCamInfo
    rightCamInfo = rightInfNow

def receiveLeftCameraInfo(leftInfNow):
    global leftCamInfo
    leftCamInfo = leftInfNow

def receiveRightFrame(rightFrameNow):
    np_arr = np.fromstring(rightFrameNow.data, np.uint8)
    rightFrameNow = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    cv2.imshow("Right window", rightFrameNow)
    cv2.waitKey(3)

def receiveLeftFrame(leftFrameNow):
    np_arr = np.fromstring(leftFrameNow.data, np.uint8)
    leftCamFrame = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    cv2.imshow("Left window", leftCamFrame)
    cv2.waitKey(3)

# Laser range-finder callback
def receiveLaser(LaserNow):
    global laserInfo 
    laserInfo = LaserNow


# IMU data callback
def receiveImu(ImuNow):
    global imuInfo 
    imuInfo = ImuNow

    
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
    std.addstr(10,0,"Actual Coils:")
    std.addstr(11, 3, "Channels: {}".format(coil.channel))
    std.addstr(12, 3, "Zeros: {}".format(coil.zero))
    std.addstr(14, 0, "Right Camera Width {} \t Height {}".format(rightCamInfo.width, rightCamInfo.height))
    std.addstr(15, 0, "Left  Camera Width {} \t Height {}".format(leftCamInfo.width, leftCamInfo.height))
    if laserInfo.ranges != []:
        std.addstr(17, 0 , "Laser Readings {} Laser Range Min {:0.4f} Laser Range Max {:0.4f}".format( len(laserInfo.ranges), min(laserInfo.ranges), max(laserInfo.ranges)))
    std.addstr(19, 0, "IMU Quaternion w: {:0.4f} x: {:0.4f} y: {:0.4f} z: {:0.4f} ".format(imuInfo.orientation.w, imuInfo.orientation.x, imuInfo.orientation.y, imuInfo.orientation.z))
    


    std.refresh()

# Basic control
def KeyCheck(stdscr):
    stdscr.keypad(True)
    stdscr.nodelay(True)

    k = None
    global std
    std = stdscr


    #publishing topics
    pubMine  = rospy.Publisher('/HRATC_FW/set_mine', Pose)
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
            pubMine.publish(pose)

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
    rospy.Subscriber("/HRATC_FW/mineDetection", Coil, receiveMineDetection)
    rospy.Subscriber("/right_camera/camera_info", CameraInfo, receiveRightCameraInfo)
    rospy.Subscriber("/left_camera/camera_info", CameraInfo, receiveLeftCameraInfo)
    rospy.Subscriber("/right_camera/image_raw/compressed", CompressedImage, receiveRightFrame)
    rospy.Subscriber("/left_camera/image_raw/compressed", CompressedImage, receiveLeftFrame)
    rospy.Subscriber("/imu_data", Imu, receiveImu)
    rospy.Subscriber("/scan", LaserScan, receiveLaser)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()

