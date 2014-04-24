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
* In this example a random walk algorithm is implemented using the ROS
* navigation stack. A random goal is set, the navigation stack manages
* path planning and obstacle avoidance.
*
*********************************************************************/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <trajectory_msgs/JointTrajectory.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RandomWalk
{
public:
    RandomWalk();
    void sendNewGoal();
    void setLimits(double min_x, double max_x, double min_y, double max_y);

private:
    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void goalActiveCallback();
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

    MoveBaseClient ac_;

    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
};

RandomWalk::RandomWalk() : ac_("move_base", true)
{
    ROS_INFO("Waiting for the move_base action server to come online...");
    if(!ac_.waitForServer(ros::Duration(5.0)))
    {
        ROS_FATAL("Did you forget to launch the ROS navigation?");
        ROS_BREAK();
    }
    ROS_INFO("Found it!");
}

void RandomWalk::sendNewGoal()
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "minefield";
    goal.target_pose.header.stamp = ros::Time::now();

    srand(time(NULL));

    double x = (double(rand()) / double(RAND_MAX)) * (max_x_-min_x_) + min_x_;
    double y = (double(rand()) / double(RAND_MAX)) * (max_y_-min_y_) + min_y_;
    double yaw = (double(rand()) / double(RAND_MAX)) * 2*M_PI - M_PI;

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    ROS_INFO("Sending a new goal to move_base x %lf y %lf yaw %lf", x, y, yaw);

    ac_.sendGoal(goal, boost::bind(&RandomWalk::goalDoneCallback, this, _1, _2), boost::bind(&RandomWalk::goalActiveCallback, this), boost::bind(&RandomWalk::goalFeedbackCallback, this, _1));
}

void RandomWalk::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{

    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The goal was reached!");

    if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
        ROS_WARN("Failed to reach the goal...");

    sendNewGoal();
}

void RandomWalk::goalActiveCallback()
{
    ROS_INFO("The new goal is active!");
}

void RandomWalk::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    //ROS_INFO("Getting feedback! How cool is that?");
}

void RandomWalk::setLimits(double min_x, double max_x, double min_y, double max_y)
{
    min_x_ = min_x;
    max_x_ = max_x;
    min_y_ = min_y;
    max_y_ = max_y;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_walk_navigation");

    ROS_INFO("HRATC 2014 random walk example - ROS Navigation version");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double min_x;
    double max_x;
    double min_y;
    double max_y;

    pn.param("min_x", min_x, -10.0);
    pn.param("max_x", max_x, 10.0);
    pn.param("min_y", min_y, -10.0);
    pn.param("max_y", max_y, 10.0);

    // Lets just make sure the laser is on the upright position, otherwise we might see the arm as an obstacle!
    ros::Publisher ptu_d46_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ptu_d46_controller/command", 10);

    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.points.resize(1);
    msg.joint_names.push_back("ptu_d46_pan_joint");
    msg.points[0].positions.push_back(0.0);
    msg.points[0].velocities.push_back(0.8);
    msg.points[0].accelerations.push_back(0.8);
    msg.joint_names.push_back("ptu_d46_tilt_joint");
    msg.points[0].positions.push_back(0.5);
    msg.points[0].velocities.push_back(0.8);
    msg.points[0].accelerations.push_back(0.8);
    msg.points[0].time_from_start = ros::Duration(1.0);

    ros::Duration(1.0).sleep();

    ptu_d46_pub.publish(msg);

    RandomWalk rw;
    rw.setLimits(min_x, max_x, min_y, max_y);
    rw.sendNewGoal();

    ros::spin();

    return 0;
}

