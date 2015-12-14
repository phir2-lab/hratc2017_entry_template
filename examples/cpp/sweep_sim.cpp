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
* In this example we show how to sweep the arm of the husky using
* an action server.
*
* With the simulator running:
* rosrun hratc2016_entry_template sweep_node
*
*********************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double min;
    pn.param("min", min, -0.8);
    double max;
    pn.param("max", max, 0.8);
    double height;
    pn.param("height", height, 0.05);
    double speed;
    pn.param("speed", speed, 0.2);
    double acceleration;
    pn.param("acceleration", acceleration, 0.5);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/arm_controller/follow_joint_trajectory", true);
    ROS_INFO("Sweep -- Waiting for the action server to start...");
    ac.waitForServer();
    ROS_INFO("Sweep -- Got it!");

    double position = min;

    while(ros::ok())
    {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.joint_names.resize(2);
        goal.trajectory.points.resize(1);
        goal.trajectory.joint_names[0] = "upper_arm_joint";
        goal.trajectory.points[0].positions.push_back(height);
        goal.trajectory.points[0].velocities.push_back(speed);
        goal.trajectory.points[0].accelerations.push_back(acceleration);
        goal.trajectory.joint_names[1] = "arm_axel_joint";
        goal.trajectory.points[0].positions.push_back(position = position == min ? max : min);
        goal.trajectory.points[0].velocities.push_back(speed);
        goal.trajectory.points[0].accelerations.push_back(acceleration);
        goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
        goal.goal_tolerance.resize(4);
        goal.goal_tolerance[0].name = "upper_arm_joint";
        goal.goal_tolerance[0].position = 0.01;
        goal.goal_tolerance[1].name = "arm_axel_joint";
        goal.goal_tolerance[1].position = 0.01;
        goal.goal_time_tolerance = ros::Duration(0.5);

        ac.sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if(!finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_ERROR("Sweep -- %s!", state.toString().c_str());
        }
    }

    return 0;
}
