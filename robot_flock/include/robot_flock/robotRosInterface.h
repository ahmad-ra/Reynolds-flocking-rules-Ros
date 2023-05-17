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

// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

#include <ros/ros.h>

#include "robot_flock/robot.h"

#include <dynamic_reconfigure/server.h>
#include <robot_flock/robotFlockConfig.h>


namespace robots
{


    class robotRosInterface
    {
    public:

        // float formationPoints[3][2]= { 0.5,0.5, 0.5,1,  1,1};
        // float formationPoints[4][2]= { 0.5,0.5, 0.5,1,  1.5,1,  1.5,0.5};
        float formationPoints[6][2]= { 1,0,2,0,2.5,1,2,2,1,2,0,1};


        ros::NodeHandle nh_;
        std::vector<ros::Publisher> vels_pub_;

        std::vector<ros::Subscriber> odom_sub_;

        std::vector<robot> robots;

        ros::Timer resetNeighborhoodsTimer;

        dynamic_reconfigure::Server<robot_flock::robotFlockConfig> server;

        robotRosInterface(int numOfRobots);

        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, const int robotId);

        void resetRobotNeighborhoods(const ros::TimerEvent &);

        void dynamicReconfigcallback(robot_flock::robotFlockConfig &config, uint32_t level);

    };

} 

#endif
