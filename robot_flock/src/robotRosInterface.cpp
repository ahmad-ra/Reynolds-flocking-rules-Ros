#include "robot_flock/robotRosInterface.h"

namespace robots
{
    robotRosInterface::robotRosInterface(int numOfRobots)
    {
        ros::Duration(2.0).sleep();

        for (int i = 0; i < numOfRobots; i++)
        {

            odom_sub_.push_back(nh_.subscribe<nav_msgs::Odometry>("robot_" + std::to_string(i) + "/odom", 100, boost::bind(&robotRosInterface::odomCallback, this, _1, i)));

            vels_pub_.push_back(nh_.advertise<geometry_msgs::Twist>("robot_" + std::to_string(i) + "/cmd_vel", 100));

            float *pos = new float[3];
            float *vel = new float[3];

            std::fill_n(pos, 3, 0.0);
            std::fill_n(vel, 3, 0.0);

            // robots.push_back(robot(i, pos, vel, 4, 4, 0.03, 1, 1));
            robots.push_back(robot(i, pos, vel, 2, 4, 1, 4, 2  ));
        }

        resetNeighborhoodsTimer = nh_.createTimer(ros::Duration(0.1), &robotRosInterface::resetRobotNeighborhoods, this);
   
    }

    void robotRosInterface::odomCallback(const nav_msgs::Odometry::ConstPtr &msg, const int robotId)
    {
        robots[robotId].position[0] = msg->pose.pose.position.x;
        robots[robotId].position[1] = msg->pose.pose.position.y;

        robots[robotId].velocity[0] = msg->twist.twist.linear.x;
        robots[robotId].velocity[1] = msg->twist.twist.linear.y;
        robots[robotId].velocity[2] = 0;

        robots[robotId].robotInitialized = true;


      
        for (int r = 0; r < robots.size(); r++)
        {
            if (robots[r].id == robotId || !robots[r].robotInitialized)
                continue;

            if (robots[r].isNeighbour(robots[robotId]) && std::find(robots[r].neighborIds->begin(), robots[r].neighborIds->end(), robots[robotId].id) == robots[r].neighborIds->end())
            {


                std::vector<robot> rr;
                rr.push_back(robots[robotId]);

                robots[r].velocity = robots[r].outputVel(std::vector<float *>{robots[r].separation(rr), robots[r].cohesion(rr), robots[r].alignment(rr)}, std::vector<float>{1, 1, 1});
                // robots[r].velocity = robots[r].outputVel(std::vector<float *>{robots[r].separation(rr),}, std::vector<float>{1});
                // robots[r].velocity = robots[r].outputVel(std::vector<float *>{robots[r].separation(rr), robots[r].cohesion(rr), robots[r].alignment(rr), robots[r].newBehevior(rr, or other parameter)}, std::vector<float>{1, 1, 1, 1});
                // robots[r].velocity = robots[r].outputVel(std::vector<float *>{ robots[r].newBehevior(rr, or other parameter)}, std::vector<float>{1}); to try your behavior alone

                robots[r].neighborIds->push_back(robotId);

      
      


                geometry_msgs::Twist twist_cmd_;
                twist_cmd_.linear.x = robots[r].velocity[0];
                twist_cmd_.linear.y = robots[r].velocity[1];
                twist_cmd_.angular.z = 0.0;
                vels_pub_[robots[r].id].publish(twist_cmd_);
            }
        }

    }

    void robotRosInterface::resetRobotNeighborhoods(const ros::TimerEvent &)
    {

        for (auto r : robots)
            r.resetNeighborhood();
    }

}