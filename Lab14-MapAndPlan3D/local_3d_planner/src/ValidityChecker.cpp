#include "ValidityChecker.h"

ValidityChecker::ValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                 std::shared_ptr <fcl::CollisionObject<double>> _robot,
                                 std::shared_ptr <fcl::CollisionObject<double>> _octreeInFCL) :
        ompl::base::StateValidityChecker(si),
        robot(_robot),
        octreeInFCL(_octreeInFCL) {
}

bool ValidityChecker::isValid(const ompl::base::State *state) const {
    // Moving robot
    virtualMove(state);

    // Checking collision
    fcl::CollisionRequest<double> fclReq(1, false, 1, false);
    fcl::CollisionResult<double> fclRes;
    fcl::collide(robot.get(), octreeInFCL.get(), fclReq, fclRes);

    return (!fclRes.isCollision());
}

double ValidityChecker::clearance(const ompl::base::State *state) const {
     // Moving robot
    virtualMove(state);

    fcl::DistanceRequest<double> fclDistReq(false, false, 0.0, 0.0, 1e-1);
    fcl::DistanceResult<double> fclDistRes;
    fcl::distance(robot.get(), octreeInFCL.get(), fclDistReq, fclDistRes);

    return fclDistRes.min_distance;
}

void ValidityChecker::virtualMove(const ompl::base::State *state) const {

    // Getting current state to check
    const ompl::base::RealVectorStateSpace::StateType *poseRealVec = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Saving as translation
    fcl::Vector3<double> t(poseRealVec->values[0], poseRealVec->values[1], poseRealVec->values[2]);

    // We assume that rotation does not matter!
    robot->setTranslation(t);
}
