#include "planningObjectives/BiasedRealVectorStateSpace.h"

double BiasedRealVectorStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const {
    double dist = 0.0;
    const double *s1 = static_cast<const RealVectorStateSpace::StateType *>(state1)->values;
    const double *s2 = static_cast<const RealVectorStateSpace::StateType *>(state2)->values;

    if(dimension_ != 3)
        return 0;

    for (unsigned int i = 0; i < dimension_; ++i) {
        double diff = (*s1++) - (*s2++);
        dist += weight[i] * diff * diff;
    }
    return sqrt(dist);
};