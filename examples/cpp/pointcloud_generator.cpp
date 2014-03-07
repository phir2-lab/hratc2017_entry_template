/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 28/02/2014
*
* This is ment to be an example file for the HRATC2104 Challenge.
* In this example we show how to control the pan and tilt module
* using an action server. This example is used to generate a 3D
* point cloud by means of the laser assembler.
*
* With the simulator running:
* roslaunch hratc2014_entry_template pointcloud_generator.launch
*
*********************************************************************/

#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_generator");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double tilt_speed;
    pn.param("tilt_speed", tilt_speed, 0.8);

    double lower_tilt;
    pn.param("lower_tilt", lower_tilt, -0.5);

    double upper_tilt;
    pn.param("upper_tilt", upper_tilt, 0.5);

    double timeout;
    pn.param("timeout", timeout, 10.0);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> tilt_ac("/ptu_d46_controller/follow_joint_trajectory", true);
    ROS_INFO("PointCloud Generator -- Waiting for tilt action server to start...");
    tilt_ac.waitForServer();
    ROS_INFO("PointCloud Generator -- Got it!");

    ROS_INFO("PointCloud Generator -- Waiting for laser assembler service to start...");
    ros::service::waitForService("assemble_scans2");
    ROS_INFO("PointCloud Generator -- Got it!");
    ros::ServiceClient client = n.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
    laser_assembler::AssembleScans2 assembler_srv;

    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1);

    // Move laser to the start position
    double tilt = upper_tilt;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.joint_names.resize(2);
    goal.trajectory.points.resize(1);
    goal.trajectory.joint_names[0] = "ptu_d46_pan_joint";
    goal.trajectory.points[0].positions.push_back(0.0);
    goal.trajectory.points[0].velocities.push_back(0.0);
    goal.trajectory.joint_names[1] = "ptu_d46_tilt_joint";
    goal.trajectory.points[0].positions.push_back(tilt);
    goal.trajectory.points[0].velocities.push_back(tilt_speed);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
    goal.goal_tolerance.resize(2);
    goal.goal_tolerance[0].name = "ptu_d46_pan_joint";
    goal.goal_tolerance[0].position = 0.01;
    goal.goal_tolerance[1].name = "ptu_d46_tilt_joint";
    goal.goal_tolerance[1].position = 0.01;
    goal.goal_time_tolerance = ros::Duration(0.5);

    tilt_ac.sendGoal(goal);

    // Wait for the action to return
    bool finished_before_timeout = tilt_ac.waitForResult(ros::Duration(timeout));

    if(!finished_before_timeout)
    {
        ROS_FATAL("PointCloud Generator -- Unable to move the laser to the start position!");
        ROS_BREAK();
    }

    while(ros::ok())
    {
        assembler_srv.request.begin = ros::Time::now();

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.joint_names.resize(2);
        goal.trajectory.points.resize(1);
        goal.trajectory.joint_names[0] = "ptu_d46_pan_joint";
        goal.trajectory.points[0].positions.push_back(0.0);
        goal.trajectory.points[0].velocities.push_back(0.0);
        goal.trajectory.joint_names[1] = "ptu_d46_tilt_joint";
        goal.trajectory.points[0].positions.push_back(tilt = tilt == upper_tilt ? lower_tilt : upper_tilt);
        goal.trajectory.points[0].velocities.push_back(tilt_speed);
        goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
        goal.goal_tolerance.resize(2);
        goal.goal_tolerance[0].name = "ptu_d46_pan_joint";
        goal.goal_tolerance[0].position = 0.01;
        goal.goal_tolerance[1].name = "ptu_d46_tilt_joint";
        goal.goal_tolerance[1].position = 0.01;
        goal.goal_time_tolerance = ros::Duration(0.5);

        tilt_ac.sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = tilt_ac.waitForResult(ros::Duration(timeout));

        if(finished_before_timeout)
        {
            assembler_srv.request.end = ros::Time::now();

            if(client.call(assembler_srv))
            {
                cloud_pub.publish(assembler_srv.response.cloud);
            }
            else
            {
                ROS_WARN("PointCloud Generator - Service call failed!");
            }
        }
        else
        {
            ROS_WARN("PointCloud Generator - ActionServer timeout!");
        }
    }

    return 0;
}
