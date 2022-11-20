#include "robot_flock/robot.h"
#include "robot_flock/robotRosInterface.h"

#include <iostream>

using namespace robots;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_flock_node");
    ros::NodeHandle nodeHandle;

    int numRobots;
    nodeHandle.getParam("numRobots", numRobots);

    robotRosInterface robot_ros_interface(numRobots);
    ros::spin();
    return 0;
}

/*

notes:

the constPtr da msh fazlaka, lakn 3shan ysht8l m3 el boost::bind
*/