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
* In this example a random walk algorithm is implemented.
* In this example we demonstrate how to subscribe to the odometry and
* laser data and publish velocity commands to the robot. Note that there
* is also a similar example that uses ROS' navigation stack, which we
* would advise over controlling the robot directly.
*
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <trajectory_msgs/JointTrajectory.h>

class Robot
{
public:
    Robot();
    void setRangeLimit(double min_range);
    void setSpeed(double linear_speed, double angular_speed);

    bool obstacle(){return obstacle_;}

    double x(){return x_;}
    double y(){return y_;}
    double yaw(){return yaw_;}

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::NodeHandle n_;

    ros::Publisher cmd_vel_pub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber odom_sub_;

    double min_range_;

    bool obstacle_;

    double x_;
    double y_;
    double yaw_;
};

Robot::Robot() : n_()
{
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/husky/cmd_vel", 10);
    laser_sub_ = n_.subscribe("scan", 10, &Robot::laserCallback, this);
    odom_sub_ = n_.subscribe("odom", 10, &Robot::odomCallback, this);

    obstacle_ = false;
}

void Robot::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    std::vector<float>::iterator range_it = std::min_element(ranges.begin(), ranges.end());

    //ROS_INFO("Smallest range is %lf", *range_it);

    if(*range_it < min_range_) obstacle_ = true;
    else obstacle_ = false;
}

void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
}

void Robot::setSpeed(double linear_speed, double angular_speed)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_speed;
    msg.angular.z = angular_speed;
    cmd_vel_pub_.publish(msg);
}

void Robot::setRangeLimit(double min_range)
{
    min_range_ = min_range;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_walk_simple");

    ROS_INFO("HRATC 2014 random walk example - Simple version");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_range;
    double max_linear_speed;
    double max_angular_speed;

    pn.param("min_x", min_x, -10.0);
    pn.param("max_x", max_x, 10.0);
    pn.param("min_y", min_y, -10.0);
    pn.param("max_y", max_y, 10.0);
    pn.param("min_range", min_range, 3.0);
    pn.param("max_linear_speed", max_linear_speed, 0.4);
    pn.param("max_angular_speed", max_angular_speed, 0.2);

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

    Robot robot;
    robot.setRangeLimit(min_range);

    double target_yaw = (double(rand()) / double(RAND_MAX)) * 2*M_PI - M_PI;

    double a = 1;

    ros::Rate r(10.0);
    while(ros::ok())
    {
        // If there is an obstacle or if we are at the edge of the admissible area set a new direction
        if(robot.obstacle() || robot.x() < min_x || robot.x() > max_x || robot.y() < min_y || robot.y() > max_y)
        {
            srand(time(NULL));
            target_yaw = angles::normalize_angle(robot.yaw()+M_PI) + ((double(rand()) / double(RAND_MAX)) * M_PI/4 - M_PI/8);
        }

        robot.setSpeed(max_linear_speed, max_angular_speed*angles::shortest_angular_distance(robot.yaw(), target_yaw)*a);
        //ROS_INFO("Linear %lf Angular %lf", max_linear_speed, max_angular_speed*angles::shortest_angular_distance(robot.yaw(), target_yaw)*a);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
