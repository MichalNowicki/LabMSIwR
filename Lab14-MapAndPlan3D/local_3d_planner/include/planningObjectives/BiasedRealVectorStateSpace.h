#ifndef BIASEDREALVECTORSTATESPACE_H
#define BIASEDREALVECTORSTATESPACE_H

#include <ompl/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

/**
 * Distance based on difference weights for planar and height motion
 */
class BiasedRealVectorStateSpace : public ompl::base::RealVectorStateSpace
{
public:
    BiasedRealVectorStateSpace(double weightZ = 1.0) : RealVectorStateSpace (3) {
        weight[0] = 1.0;
        weight[1] = 1.0;
        weight[2] = weightZ;
    }

    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

private:
    double weight[3];

};

#endif