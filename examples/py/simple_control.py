#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# read/write stuff on screen
std = None  

# Robot data
robotTwist = Twist()
robotOdom = Odometry()
radStep = deg2rad(15)
linStep = 0.1

# Position callback
def receiveOdom(actualOdometry):
    global robotOdom 
    robotOdom = actualOdometry
    showStats()

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
    std.addstr(7,0,"Actual Position:")
    std.addstr(8, 0, "{} \t {} \t {}".format(robotOdom.pose.pose.position.x,robotOdom.pose.pose.position.y,robotOdom.pose.pose.position.z))

    std.refresh()

# Basic control
def KeyCheck(stdscr):
    stdscr.keypad(True)
    stdscr.nodelay(True)

    k = None
    global std
    std = stdscr

    #publishing topics
    pubVel   = rospy.Publisher('/sim_p3at/cmd_vel', Twist)

    # While 'Esc' is not pressed
    while k != chr(27):
        # Check no key
        try:
            k = stdscr.getkey()
        except:
            k = None

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
    
# Initialize curses stuff
def StartControl():
    wrapper(KeyCheck)

# ROSPy stuff
def spin():
    rospy.spin()

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('client')

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/sim_p3at/odom", Odometry, receiveOdom)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()

