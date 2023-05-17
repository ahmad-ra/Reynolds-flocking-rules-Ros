#include "robot_flock/robotRosInterface.h"

namespace robots
{
    robotRosInterface::robotRosInterface(int numOfRobots)
    {
        ros::Duration(2.0).sleep();

        // Eigen::MatrixXd formationPoints(numOfRobots 2);
        // formationPoints << ; // from SO: Performance-wise, the compiler is able to optimize both the same. Checked it with GCC.

        // float formationPoints[numOfRobots][2]= { 0,0, 0,1,0.5,1};

        for (int i = 0; i < numOfRobots; i++)
        {

            odom_sub_.push_back(nh_.subscribe<nav_msgs::Odometry>("robot_" + std::to_string(i) + "/odom", 100, boost::bind(&robotRosInterface::odomCallback, this, _1, i)));

            vels_pub_.push_back(nh_.advertise<geometry_msgs::Twist>("robot_" + std::to_string(i) + "/cmd_vel", 100));

            float *pos = new float[3];
            float *vel = new float[3];
            float *formationPoint = new float[2];
            formationPoint[0] = formationPoints[i][0];
            formationPoint[1] = formationPoints[i][1];

            std::fill_n(pos, 3, 0.0);
            std::fill_n(vel, 3, 0.0);

            // robots.push_back(robot(i, pos, vel, 4, 4, 0.03, 1, 1));
            // robots.push_back(robot(i, pos, vel, 2, 4, 1, 4, 2));
            robots.push_back(robot(i, pos, vel, 2, 4, 0.1, 0, 0, 0.5, formationPoint));
        }

        resetNeighborhoodsTimer = nh_.createTimer(ros::Duration(0.1), &robotRosInterface::resetRobotNeighborhoods, this);

        dynamic_reconfigure::Server<robot_flock::robotFlockConfig>::CallbackType cb;
        cb = boost::bind(&robotRosInterface::dynamicReconfigcallback, this, _1, _2);

        server.setCallback(cb);
    }

    void robotRosInterface::odomCallback(const nav_msgs::Odometry::ConstPtr &msg, const int robotId)
    {
        robots[robotId].position[0] = msg->pose.pose.position.x;
        robots[robotId].position[1] = msg->pose.pose.position.y;

        tf::Quaternion Q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, Q);
        robots[robotId].position[2] = tf::getYaw(Q);

        // tranform velocity from robot frame to world frame
        Eigen::MatrixXd rotMatrix2World(2, 2);
        rotMatrix2World << cos(robots[robotId].position[2]), -sin(robots[robotId].position[2]), sin(robots[robotId].position[2]), cos(robots[robotId].position[2]); // from SO: Performance-wise, the compiler is able to optimize both the same. Checked it with GCC.
        // std::cout<<rotMatrix2World<<std::endl;
        Eigen::Vector2d vel = rotMatrix2World * Eigen::Vector2d(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

        // robots[robotId].velocity[0] = msg->twist.twist.linear.x;
        // robots[robotId].velocity[1] = msg->twist.twist.linear.y;
        robots[robotId].velocity[0] = vel(0);
        robots[robotId].velocity[1] = vel(1);
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

                // robots[r].velocity = robots[r].outputVel(std::vector<float *>{robots[r].formationConsensus(rr)}, std::vector<float>{1});
                robots[r].velocity = robots[r].outputVel(std::vector<float *>{robots[r].separation(rr), robots[r].cohesion(rr), robots[r].alignment(rr), robots[r].formationConsensus(rr)}, std::vector<float>{1, 1, 1, 1});
                // robots[r].velocity = robots[r].outputVel(std::vector<float *>{robots[r].separation(rr), robots[r].cohesion(rr), robots[r].alignment(rr)}, std::vector<float>{1, 1, 1});
                // robots[r].velocity = robots[r].outputVel(std::vector<float *>{robots[r].separation(rr),}, std::vector<float>{1});
                // robots[r].velocity = robots[r].outputVel(std::vector<float *>{robots[r].separation(rr), robots[r].cohesion(rr), robots[r].alignment(rr), robots[r].newBehevior(rr, or other parameter)}, std::vector<float>{1, 1, 1, 1});
                // robots[r].velocity = robots[r].outputVel(std::vector<float *>{ robots[r].newBehevior(rr, or other parameter)}, std::vector<float>{1}); to try your behavior alone

                robots[r].neighborIds->push_back(robotId);

                // transform velocity from world frame to the robot frame
                // rRotw * Vel
                Eigen::MatrixXd rotMatrix(2, 2);
                rotMatrix << cos(robots[r].position[2]), sin(robots[r].position[2]), -sin(robots[r].position[2]), cos(robots[r].position[2]);
                // std::cout<<rotMatrix<<std::endl;
                Eigen::Vector2d res = rotMatrix * Eigen::Vector2d(robots[r].velocity[0], robots[r].velocity[1]);
                // std::cout<<Eigen::Vector2d( robots[r].velocity[0], robots[r].velocity[1] )<<std::endl;
                // std::cout << "id: " << robots[r].id << std::endl;

                // std::cout << res << std::endl;

                geometry_msgs::Twist twist_cmd_;
                // twist_cmd_.linear.x = robots[r].velocity[0];
                // twist_cmd_.linear.y = robots[r].velocity[1];
                twist_cmd_.linear.x = res(0);
                twist_cmd_.linear.y = res(1);
                twist_cmd_.angular.z = 0.0;
                vels_pub_[robots[r].id].publish(twist_cmd_);
            }
        }
    }

    void robotRosInterface::dynamicReconfigcallback(robot_flock::robotFlockConfig &config, uint32_t level)
    {

        odom_sub_.clear();
        vels_pub_.clear();
        robots.clear();

        for (int i = 0; i < config.numRobots; i++)
        {

            odom_sub_.push_back(nh_.subscribe<nav_msgs::Odometry>("robot_" + std::to_string(i) + "/odom", 100, boost::bind(&robotRosInterface::odomCallback, this, _1, i)));

            vels_pub_.push_back(nh_.advertise<geometry_msgs::Twist>("robot_" + std::to_string(i) + "/cmd_vel", 100));

            float *pos = new float[3];
            float *vel = new float[3];
            float *formationPoint = new float[2];
            formationPoint[0] = formationPoints[i][0];
            formationPoint[1] = formationPoints[i][1];

            std::fill_n(pos, 3, 0.0);
            std::fill_n(vel, 3, 0.0);

            robots.push_back(robot(i, pos, vel, config.radius, 4, config.separationStrength, config.cohesionStrength, config.alignmentStrength, config.formationStepSize, formationPoint));
        }
    }

    void robotRosInterface::resetRobotNeighborhoods(const ros::TimerEvent &)
    {

        geometry_msgs::Twist twist_cmd_;
        twist_cmd_.linear.x = 0.0;
        twist_cmd_.linear.y = 0.0;
        twist_cmd_.angular.z = 0.0;

        for (auto r : robots)
        {
            r.resetNeighborhood();

            vels_pub_[r.id].publish(twist_cmd_);
        }
    }

}