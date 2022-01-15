#ifndef PLANNINGOBJECTIVES_H
#define PLANNINGOBJECTIVES_H

#include <ompl/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include "BiasedRealVectorStateSpace.h"

ompl::base::OptimizationObjectivePtr getPathLengthObjective(const ompl::base::SpaceInformationPtr &si,
                                                            const double earlyEndCostThreshold);


#endif 
