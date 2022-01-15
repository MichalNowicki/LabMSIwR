#include "planningObjectives/PlanningObjectives.h"

ompl::base::OptimizationObjectivePtr getPathLengthObjective(const ompl::base::SpaceInformationPtr &si,
                                                            const double earlyEndCostThreshold) {
    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::PathLengthOptimizationObjective(si));
    lengthObj->setCostThreshold(ompl::base::Cost(earlyEndCostThreshold));
    return lengthObj;
}