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
* In this example we show how to tell the HRATC 2014 Judge that we
* found a mine!
*
* With the simulator running:
* rosrun hratc2016_entry_template set_mine _x:=5.0 _y:=3.0
*
*********************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_mine_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double x;
    pn.param("x", x, 0.0);
    double y;
    pn.param("y", y, 0.0);
    std::string frame_id;
    pn.param<std::string>("frame_id", frame_id, "minefield");

    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/HRATC_FW/set_mine", 1);

    // We're just waiting for a subscriber, otherwise we would be publishing our first and only message to oblivion!
    ros::Rate poll_rate(10);
    while(pub.getNumSubscribers() == 0)
        poll_rate.sleep();

    geometry_msgs::PoseStamped mine_pose;
    mine_pose.header.stamp = ros::Time::now();
    mine_pose.header.frame_id = frame_id;
    // We're actually only using a point x y...
    mine_pose.pose.position.x = x;
    mine_pose.pose.position.y = y;
    mine_pose.pose.position.z = 0.0;
    mine_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    ROS_INFO("Setting a mine on x:%lf y:%lf", x, y);

    pub.publish(mine_pose);

    ROS_INFO("Press Ctrl+C to exit.");

    ros::spin();

    return 0;
}
