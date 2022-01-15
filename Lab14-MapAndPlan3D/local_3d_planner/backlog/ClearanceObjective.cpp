#include "planningObjectives/ClearanceObjective.h"

ClearanceObjective::ClearanceObjective(const ompl::base::SpaceInformationPtr &si) :
        ompl::base::StateCostIntegralObjective(si, true) {
}

ompl::base::Cost ClearanceObjective::stateCost(const ompl::base::State *s) const {
    double distance = si_->getStateValidityChecker()->clearance(s);
    double cost = 0;
    if (distance < 3)
        cost = 1.0 / distance;

    return ompl::base::Cost(cost);
}

/** \brief Allocate a state sampler for the path-length objective (i.e., direct ellipsoidal sampling). */
ompl::base::InformedSamplerPtr ClearanceObjective::allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
                                              unsigned int maxNumberCalls) const {

    return std::make_shared<ompl::base::PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
}
