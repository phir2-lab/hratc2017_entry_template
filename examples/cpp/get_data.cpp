#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <metal_detector_msgs/Coil.h>

#include <iostream>
using namespace std;

class Robot
{
public:
    Robot() : tf_()
    {
        sub_Odom = n_.subscribe("/p3at/odom", 10, &Robot::odomCallback, this);
        sub_GPS = n_.subscribe("/gps/odom", 10, &Robot::gpsCallback, this);
        sub_Coils = n_.subscribe("/coils", 10, &Robot::coilsCallback, this);


        emptyPose.header.frame_id = "UNDEF";
        emptyPose.pose.position.x = 0;
        emptyPose.pose.position.y = 0;
        emptyPose.pose.position.z = 0;
        quaternionTFToMsg(tf::createQuaternionFromYaw(0.0*M_PI/180.0),
                              emptyPose.pose.orientation);
    }

    void run()
    {
        ros::Rate r(20);

        while (ros::ok())
        {
            cout << "ODOM x:" << odom.pose.pose.position.x
                 << " y:" << odom.pose.pose.position.y
                 << " yaw:" << tf::getYaw(odom.pose.pose.orientation) << endl;

            cout << "GPS x:" << gps.pose.pose.position.x
                 << " y:" << gps.pose.pose.position.y
                 << " yaw:" << tf::getYaw(gps.pose.pose.orientation) << endl;

            geometry_msgs::PoseStamped robotPoseFromTF = getRobotPoseFromTF();
            cout << "POSE from TF x:" << robotPoseFromTF.pose.position.x
                 << " y:" << robotPoseFromTF.pose.position.y
                 << " yaw:" << tf::getYaw(robotPoseFromTF.pose.orientation) << endl;

            geometry_msgs::PoseStamped leftCoilPose = getLeftCoilPose(robotPoseFromTF);
            cout << "COIL pose - x:" << leftCoilPose.pose.position.x
                 << " y:" << leftCoilPose.pose.position.y
                 << " -> value:" << coils.left_coil << endl;

            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_Odom, sub_GPS, sub_Coils;
    tf::TransformListener tf_;

    nav_msgs::Odometry odom;
    nav_msgs::Odometry gps;
    metal_detector_msgs::Coil coils;

    geometry_msgs::PoseStamped emptyPose;

    void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        odom.header.frame_id = msg->header.frame_id;
        odom.header.stamp = msg->header.stamp;
        odom.pose = msg->pose;
    }

    void gpsCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        gps.header.frame_id = msg->header.frame_id;
        gps.header.stamp = msg->header.stamp;
        gps.pose = msg->pose;
    }

    void coilsCallback(const metal_detector_msgs::Coil::ConstPtr & msg)
    {
        coils.left_coil = msg->left_coil;
        coils.right_coil = msg->right_coil;
    }

    geometry_msgs::PoseStamped getRobotPoseFromTF()
    {
        geometry_msgs::PoseStamped robotPose;
        tf::StampedTransform robotTransform;

        ros::Time now = ros::Time::now();
        try{
            tf_.waitForTransform("/minefield", "/base_link", now, ros::Duration(2.0));
            tf_.lookupTransform("/minefield", "/base_link", now, robotTransform);
        }
        catch (tf::TransformException &ex) {
    //        ROS_ERROR("%s",ex.what());
            return emptyPose;
        }

        robotPose.header.frame_id = "base_link";
        robotPose.header.stamp = ros::Time::now();
        robotPose.pose.position.x = robotTransform.getOrigin().x();
        robotPose.pose.position.y = robotTransform.getOrigin().y();
        robotPose.pose.position.z = robotTransform.getOrigin().z();
        quaternionTFToMsg(robotTransform.getRotation(),robotPose.pose.orientation);

        return robotPose;
    }

    geometry_msgs::PoseStamped getLeftCoilPose(geometry_msgs::PoseStamped robotPose)
    {
        geometry_msgs::PoseStamped leftCoilPose_;
        tf::StampedTransform coilTransform;

        ros::Time now = ros::Time::now();
        try{
            tf_.waitForTransform("/base_link", "/left_coil", now, ros::Duration(2.0));
            tf_.lookupTransform("/base_link", "/left_coil", now, coilTransform);
        }
        catch (tf::TransformException &ex) {
    //        ROS_ERROR("%s",ex.what());
            return emptyPose;
        }

        // Use reference robot pose
        tf::Transform robotTransform;
        robotTransform.setOrigin( tf::Vector3(robotPose.pose.position.x,
                                              robotPose.pose.position.y,
                                              robotPose.pose.position.z) );
        tf::Quaternion q;
        quaternionMsgToTF(robotPose.pose.orientation,q);
        robotTransform.setRotation(q);

        // Compute corrected coil pose
        tf::Transform Result;
        Result.mult(robotTransform, coilTransform);

        leftCoilPose_.header.frame_id = "left_coil";
        leftCoilPose_.header.stamp = ros::Time::now();
        leftCoilPose_.pose.position.x = Result.getOrigin().x();
        leftCoilPose_.pose.position.y = Result.getOrigin().y();
        leftCoilPose_.pose.position.z = Result.getOrigin().z();
        quaternionTFToMsg(Result.getRotation(),leftCoilPose_.pose.orientation);

        return leftCoilPose_;
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Robot");

    Robot myPose;
    myPose.run();
    return 0;
};

