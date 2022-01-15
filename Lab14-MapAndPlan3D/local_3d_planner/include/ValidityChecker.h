#ifndef VALIDITYCHECKER_H
#define VALIDITYCHECKER_H

#include <ompl/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>

/**
 * Checking collisions
 */
class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<fcl::CollisionObject<double>> _robot,
                    std::shared_ptr<fcl::CollisionObject<double>> _octreeInFCL);

    /**
     * Checking collision for the current state
     */
    bool isValid(const ompl::base::State* state) const;

    /**
     * Distance to the closest obstacles
     */
    double clearance(const ompl::base::State* state) const;

    /**
     * Virtual motion to check collision
     */
    void virtualMove(const ompl::base::State *state) const;

private:
    std::shared_ptr<fcl::CollisionObject<double>> octreeInFCL;
    std::shared_ptr<fcl::CollisionObject<double>> robot;
};

#endif 