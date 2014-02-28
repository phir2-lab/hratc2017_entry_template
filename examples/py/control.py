#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, rospy, curses
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose
from mine_detection.msg._Coil import Coil
from numpy import deg2rad


t = Twist()
pose = Pose()
coil = Coil()
std = None  
radStep = deg2rad(5)


def receiveMineDetection(actualCoil):
    global coil
    coil = actualCoil


def receivePosition(actualPose):
    global pose
    pose = actualPose
    showStats()


def showStats():
    if std == None:
        return
    std.clear()
    std.addstr(0,0,"Linear:")
    std.addstr(1, 0, "{} \t {} \t {}".format(t.linear.x,t.linear.y,t.linear.z))
    std.addstr(3,0,"Angular:")
    std.addstr(4, 0, "{} \t {} \t {}".format(t.angular.x,t.angular.y,t.angular.z))
    std.addstr(6,0,"Actual Position:")
    std.addstr(7, 0, "{} \t {} \t {}".format(pose.position.x,pose.position.y,pose.position.z))
    std.addstr(9,0,"Actual Coils:")
    std.addstr(10, 3, "Channels: {}".format(coil.channel))
    std.addstr(11, 3, "Zeros: {}".format(coil.zero))
    std.refresh()


def KeyCheck(stdscr):
    stdscr.keypad(True)
    k = None
    global std
    std = stdscr
    pubMine = rospy.Publisher('/HRATC_FW/set_mine', Pose)
    pubVel = rospy.Publisher('/husky/cmd_vel', Twist)
    while k != chr(27):
        k = stdscr.getkey()

        if k == "KEY_LEFT":
            t.angular.z += radStep
        if k == "KEY_RIGHT":
            t.angular.z -= radStep
        if k == "KEY_UP":
            t.linear.x += 0.05
        if k == "KEY_DOWN":
            t.linear.x -= 0.05
        if k == "x":
            pubMine.publish(pose)
        t.angular.z = min(t.angular.z,deg2rad(90))
        t.angular.z = max(t.angular.z,deg2rad(-90))

        pubVel.publish(t)
        showStats()

    stdscr.keypad(False)
    rospy.signal_shutdown("Shutdown Competitor")


def StartControl():
    wrapper(KeyCheck)


def spin():
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Competitor')
    rospy.Subscriber("/HRATC_FW/pose", Pose, receivePosition)
    rospy.Subscriber("/HRATC_FW/mineDetection", Coil, receiveMineDetection)
    Thread(target = StartControl).start()
    Thread(target = spin).start()

