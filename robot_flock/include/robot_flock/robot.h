#ifndef ROBOT_H
#define ROBOT_H

#include <algorithm>
#include <vector>

#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>

#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>



namespace robots
{


    class robot
    {
    public:
        int id;
        bool robotInitialized{false};

        float *position, *velocity;

        // region of neigborhood
        float radius, theta;

        // general params

        std::vector<int>* neighborIds;

        // separation params
        float separationStrength;
        float *separationVel;

        // cohesion parameters
        float cohesionStrength;
        float *cohesionVel, *cohesionTermAccumulator;

        // alignment terms
        float alignmentStrength;
        float *alignmentVel, *alignmentTermAccumulator;

        // add parames of new behavior
        //
        //


        // add parames of new behavior
        //
        //

        robot();
        robot(float id, float position[], float velocity[], float radius, float theta, float separationStrength, float cohesionStrength, float alignmentStrength);

        bool isNeighbour(robot r);

        float *separation(std::vector<robot> robots);
        float *cohesion(std::vector<robot> robots);
        float *alignment(std::vector<robot> robots);
        float *outputVel(std::vector<float *> velocities, std::vector<float> percentages);

        void resetNeighborhood();
    };

} 


#endif