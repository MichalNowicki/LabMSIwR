#ifndef CUSTOMPLANNINGOBJECTIVE_H
#define CUSTOMPLANNINGOBJECTIVE_H

#include <ompl/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>

/**
 * Distance as sum of path and object clearance
 */
class PathLengthAndClearanceObjective : public ompl::base::MultiOptimizationObjective {
public:
    PathLengthAndClearanceObjective(const ompl::base::SpaceInformationPtr &si) : ompl::base::MultiOptimizationObjective(si) {
        description_ = "Path Length and Clearance";

        // Setup a default cost-to-go heuristics:
        setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
    }

    /** \brief the motion cost heuristic for this objective is
        simply the configuration space distance between \e s1
        and \e s2, since this is the optimal cost between any
        two states assuming no obstacles. */
    ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        ompl::base::Cost(si_->distance(s1, s2));
    }

    /** \brief Allocate a state sampler for the path-length objective (i.e., direct ellipsoidal sampling). */
    ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
                                                 unsigned int maxNumberCalls) const override {
        return std::make_shared<ompl::base::PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
    }

};

#endif 
