#include "robot_flock/robot.h"

namespace robots
{
    robot::robot() {}
    robot::robot(float id, float position[], float velocity[], float radius, float theta, float separationStrength, float cohesionStrength, float alignmentStrength, float formationStepSize, float *formationPoint) : id(id), position(position), velocity(velocity), radius(radius), theta(theta), separationStrength(separationStrength), cohesionStrength(cohesionStrength), alignmentStrength(alignmentStrength), formationStepSize(formationStepSize), formationPoint(formationPoint)
    {

        neighborIds = new std::vector<int>;

        separationVel = new float[3];

        cohesionVel = new float[3];
        cohesionTermAccumulator = new float[3];

        alignmentVel = new float[3];
        alignmentTermAccumulator = new float[3];

        formationVel = new float[3];
        // formationPoint = new float[2];

        for (int i = 0; i < 3; i++)
        {
            separationVel[i] = 0;
            cohesionVel[i] = 0;
            cohesionTermAccumulator[i] = 0;
            alignmentVel[i] = 0;
            alignmentTermAccumulator[i] = 0;

            formationVel[i] = 0;
        }
    }

    bool robot::isNeighbour(robot r)
    {
        float radiusDiff = sqrt(pow(r.position[0] - position[0], 2) + pow(r.position[1] - position[1], 2));

        // float thetaDiff = position[2] - r.position[2];

        // return ((radiusDiff <= radius) & (thetaDiff <= theta));
        return ((radiusDiff <= radius));
    }

    float *robot::separation(std::vector<robot> robots)
    {
        auto separationTerm = std::make_unique<float[]>(3);
        separationTerm[0] = 0.0;
        separationTerm[1] = 0.0;
        // separationTerm[2] = 0.0;
        for (auto r : robots)
        {

            // separationTerm[0] += separationStrength / ((position[0] - r.position[0]));
            // separationTerm[1] += separationStrength / ((position[1] - r.position[1]));

            // separationTerm[0] += separationStrength / ((position[0] - r.position[0]) * (position[0] - r.position[0]));
            // separationTerm[1] += separationStrength / ((position[1] - r.position[1]) * (position[1] - r.position[1]));

            separationTerm[0] += ((position[0] - r.position[0]));
            separationTerm[1] += ((position[1] - r.position[1]));
        }

        float norm = sqrt(pow(separationTerm[0], 2) + pow(separationTerm[1], 2));
        // for (int i = 0; i < 3; i++)
        // {

        //     separationTerm[i] /= norm;
        // }

        for (int i = 0; i < 3; i++)
        {

            separationTerm[i] *= separationStrength / ((norm * norm));
        }

        if (norm < 0.2)
        {
            separationVel[0] += (isinf(separationTerm[0])) ? 0.0 : separationTerm[0];
            separationVel[1] += (isinf(separationTerm[1])) ? 0.0 : separationTerm[1];
        }
        // std::cout<<norm<<std::endl;
        // if ( sqrt(pow(separationTerm[0], 2) + pow(separationTerm[1], 2)) < 50)
        // {
        //     separationVel[0] += 0;
        //     separationVel[1] += 0;
        // }

        // separationVel[0] += (isinf(separationTerm[0]) || isinf(separationTerm[1])) ? 0.0 : copysignf(1.0, separationTerm[0]) * abs(separationTerm[1]);
        // separationVel[1] += (isinf(separationTerm[1]) || isinf(separationTerm[0])) ? 0.0 : copysignf(1.0, separationTerm[1]) * abs(separationTerm[0]);

        return separationVel;
    }

    float *robot::cohesion(std::vector<robot> robots)
    {

        // auto cohesionTerm = std::make_unique<float[]>(3);

        // cohesionTerm[0] = 0.0;
        // cohesionTerm[1] = 0.0;
        // cohesionTerm[2] = 0.0;

        int addedRobots = 0;
        for (auto r : robots)
        {

            if (std::find(neighborIds->begin(), neighborIds->end(), r.id) != neighborIds->end())
            {

                continue;
            }
            addedRobots++;

            cohesionTermAccumulator[0] += (r.position[0]);
            cohesionTermAccumulator[1] += (r.position[1]);
        }

        if (addedRobots > 0)
        {
            cohesionVel[0] = cohesionStrength * ((cohesionTermAccumulator[0] / (neighborIds->size() + addedRobots)) - position[0]);
            cohesionVel[1] = cohesionStrength * ((cohesionTermAccumulator[1] / (neighborIds->size() + addedRobots)) - position[1]);
        }

        return cohesionVel;
    }

    float *robot::alignment(std::vector<robot> robots)
    {

        auto alignmentTerm = std::make_unique<float[]>(3);

        alignmentTerm[0] = 0.0;
        alignmentTerm[1] = 0.0;
        // alignmentTerm[2] = 0.0;

        int addedRobots = 0;
        for (auto r : robots)
        {
            if (std::find(neighborIds->begin(), neighborIds->end(), r.id) != neighborIds->end())
                continue;

            addedRobots++;

            alignmentTermAccumulator[0] += (r.velocity[0]);
            alignmentTermAccumulator[1] += (r.velocity[1]);
            // alignmentTermAccumulator[2] +=  (r.velocity[2]);
        }

        if (addedRobots > 0)
        {
            alignmentVel[0] = alignmentStrength * alignmentTermAccumulator[0] / (neighborIds->size() + addedRobots);
            alignmentVel[1] = alignmentStrength * alignmentTermAccumulator[1] / (neighborIds->size() + addedRobots);
            // alignmentVel[2] = alignmentStrength * alignmentTermAccumulator[2] / (neighborIds->size() + addedRobots);
        }

        return alignmentVel;
    }

    float *robot::formationConsensus(std::vector<robot> robots)
    {

        for (auto r : robots)
        {
            if (std::find(neighborIds->begin(), neighborIds->end(), r.id) != neighborIds->end())
                continue;

            formationVel[0] += formationStepSize * ((r.position[0] - r.formationPoint[0]) - (position[0] - formationPoint[0]));
            formationVel[0] = abs(formationVel[0]) < 0.05 ? 0 : formationVel[0];

            formationVel[1] += formationStepSize * ((r.position[1] - r.formationPoint[1]) - (position[1] - formationPoint[1]));
            formationVel[1] = abs(formationVel[1]) < 0.05 ? 0 : formationVel[1];

            // formationVel[0]+=formationStepSize*(( r.formationVel[0] - formations[j,0] ) - (formationVel[0] - formations[i,0] ));
            // formationVel[1]+=formationStepSize*(( r.formationVel[1] - formations[j,1] ) - (formationVel[1] - formations[i,1] ));
            // std::cout<< formationVel[0]<<" and "<< formationVel[1] <<std::endl;
        }

        return formationVel;
    }

    float *robot::outputVel(std::vector<float *> velocities, std::vector<float> percentages)
    {
        float *outVel = new float[3];

        outVel[0] = 0.0;
        outVel[1] = 0.0;
        outVel[2] = 0.0;

        for (int i = 0; i < percentages.size(); i++)
        {
            // norm = sqrt(pow(velocities[i][0], 2) + pow(velocities[i][1], 2) + pow(velocities[i][2], 2));

            for (int j = 0; j < 3; j++)
            {

                outVel[j] += velocities[i][j];
            }
        }

        // float norm = sqrt(pow(outVel[0], 2) + pow(outVel[1], 2) + pow(outVel[2], 2));
        float norm = sqrt(pow(outVel[0], 2) + pow(outVel[1], 2));

        if (norm > 0)
        {
            for (int i = 0; i < 3; i++)
            {

                outVel[i] /= norm;
            }
        }

        // for (int i = 0; i < 3; i++)
        // {

        //     outVel[i] = outVel[i] > percentages[i] ? percentages[i] : outVel[i];
        // }

        // norm = sqrt(pow(outVel[0], 2) + pow(outVel[1], 2) + pow(outVel[2], 2));
        // for (int i = 0; i < 3; i++)
        // {

        //     outVel[i] /= norm;
        // }
        velocity[0] = outVel[0];
        velocity[1] = outVel[1];
        // velocity[0]=velocity[0]<0.001?0:velocity[0];
        // velocity[1]=velocity[1]<0.001?0:velocity[1];
        // velocity[2] = outVel[2];

        delete[] outVel;
        return velocity;
    }

    void robot::resetNeighborhood()
    {
        std::fill_n(separationVel, 3, 0.0);

        std::fill_n(cohesionVel, 3, 0.0);
        std::fill_n(cohesionTermAccumulator, 3, 0.0);

        std::fill_n(alignmentVel, 3, 0.0);
        std::fill_n(alignmentTermAccumulator, 3, 0.0);

        std::fill_n(formationVel, 3, 0.0);

        neighborIds->clear();
    }

}