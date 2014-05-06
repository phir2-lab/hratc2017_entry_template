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
* In this example we show how to read from the metal detector
* and index the readings to a different referential than that of
* the coils.
*
* Based on
* http://wiki.ros.org/tf/Tutorials/Using%20Stamped%20datatypes%20with%20tf%3A%3AMessageFilter
*
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class GlobalPose
{
public:
    GlobalPose() : tf_(),  target_frame_("minefield")
    {
        sub_.subscribe(n_, "/robot_pose_ekf/odom", 10);
        tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&GlobalPose::msgCallback, this, _1) );
    }

private:
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped> * tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;

    //  Callback to register with tf::MessageFilter to be called when transforms are available
    void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped>& msg)
    {
        geometry_msgs::PoseStamped pose_in;
        pose_in.header.frame_id = msg->header.frame_id;
        pose_in.header.stamp = msg->header.stamp;
        pose_in.pose.position.x = msg->pose.pose.position.x;
        pose_in.pose.position.y = msg->pose.pose.position.y;
        pose_in.pose.position.z = msg->pose.pose.position.z;
        pose_in.pose.orientation = msg->pose.pose.orientation;

        geometry_msgs::PoseStamped pose_out;
        try
        {
            tf_.transformPose(target_frame_, pose_in, pose_out);

            ROS_INFO("Robot is at x:%lf y:%lf yaw:%lf", pose_out.pose.position.x, pose_out.pose.position.y, tf::getYaw(pose_out.pose.orientation));
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "get_global_odom");

    GlobalPose myPose;

    ros::spin();

    return 0;
};

