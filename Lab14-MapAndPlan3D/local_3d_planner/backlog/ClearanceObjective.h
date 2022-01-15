#ifndef CLEARANCEOBJECTIVE_H
#define CLEARANCEOBJECTIVE_H

#include <ompl/config.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>

/**
 * Path planning based on distance to obstacles
 */
class ClearanceObjective : public ompl::base::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ompl::base::SpaceInformationPtr& si);

    ompl::base::Cost stateCost(const ompl::base::State* s) const;


    /** \brief Allocate a state sampler for the path-length objective (i.e., direct ellipsoidal sampling). */
    ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
                                                             unsigned int maxNumberCalls) const override;
};



#endif
