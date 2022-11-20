#ifndef ROBOT_ROS_INTERFACE_H
#define ROBOT_ROS_INTERFACE_H

#include <algorithm>
#include <vector>

#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>

#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

#include <ros/ros.h>

#include "robot_flock/robot.h"



namespace robots
{


    class robotRosInterface
    {
    public:

        ros::NodeHandle nh_;
        std::vector<ros::Publisher> vels_pub_;

        std::vector<ros::Subscriber> odom_sub_;

        std::vector<robot> robots;

        ros::Timer resetNeighborhoodsTimer;

        robotRosInterface(int numOfRobots);

        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, const int robotId);

        void resetRobotNeighborhoods(const ros::TimerEvent &);
    };

} 

#endif
