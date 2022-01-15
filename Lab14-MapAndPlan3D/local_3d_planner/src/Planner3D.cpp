#include "Planner3D.h"
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>

Planner3D::Planner3D() : nh("~") {
    ROS_INFO_STREAM("Planner3D::Planner3D() - start");

    // Planning parameters
    planningTimeInitial = readParameter(nh, TAG, "planning_time_initial", 5);
    planningTimeSecondary = readParameter(nh, TAG, "planning_time_secondary", 60);
    planningTerminateCondCheckInterval = readParameter(nh, TAG, "planning_terminate_cond_check_interval", 1);
    planningObjectiveZWeight = readParameter(nh, TAG, "planning_objective_z_weight", 1);

    // Reading the names of the topics
    std::string goalTopic = readParameter<std::string>(nh, TAG, "goalTopic", "/move_base_simple/goal");
    std::string poseTopic = readParameter<std::string>(nh, TAG, "poseTopic", "/odom");
    std::string octreeTopic = readParameter<std::string>(nh, TAG, "octreeTopic", "/octomap_full");
    
    
    std::string plannerStringTopic = readParameter<std::string>(nh, TAG, "plannerStringTopic",
                                                                "/local_planner/planner_info");
    std::string plannedPathMarkersTopic = readParameter<std::string>(nh, TAG, "plannedPathMarkersTopic",
                                                                     "/local_planner/planned_path_markers");


    /// Subscribers
    subscriberGoal = nh.subscribe<geometry_msgs::PoseStamped>(goalTopic, 2, &Planner3D::goalCallback, this);
    subscriberPose = nh.subscribe(poseTopic, 2, &Planner3D::poseCallback, this);
    subscriberOctree = nh.subscribe("/octomap_full", 2, &Planner3D::octreeCallback, this);

    // Publishing planned path
    publisherPlannerString = nh.advertise<std_msgs::String>(plannerStringTopic, 0);
    // publisherPlannedPath = nh.advertise<nav_msgs::Path>(plannedPathTopic, 0);
    publisherPlannedPathMarkers = nh.advertise<visualization_msgs::Marker>(plannedPathMarkersTopic, 0);

    // Creating robot
    double robotSizeXY = readParameter(nh, TAG, "robotSizeXY", 0.5);
    double robotSizeZ = readParameter(nh, TAG, "robotSizeZ", 0.2);
    robot = std::make_shared < fcl::CollisionObject < double >> (std::shared_ptr < fcl::CollisionGeometry < double
            >> (new fcl::Box<double>(robotSizeXY, robotSizeXY, robotSizeZ)));

    // Initialize octree
    octree.reset(new octomap::OcTree(0.1));
    
    // Initial start and goal locations
    lastRobotLocation = Eigen::Vector3d::Zero();
    goalLocation = Eigen::Vector3d::Zero();
    
    // Variables informing if new data has arrived
    updatedGoal = false;

    // Initialize interactive markers
    breakPlanning = false;

    lastOctreeUpdate = ros::Time::now();
    emptyOctree = true;

    // Initializing main processing thread
    planningThreadRun = true;
    mainThreadPtr.reset(new std::thread(&Planner3D::run, this));
}

Planner3D::~Planner3D() {
    planningThreadRun = false;
    mainThreadPtr.get()->join();

    octreeUpdateThreadPtr.get()->join();
    planThreadPtr.get()->join();
}


void Planner3D::run() {
    std::cout << "Planner3D::run() - begin!" << std::endl;

    while (planningThreadRun) {
        bool performedProcessing = false;
    
        // We received new goal - let's plan
        if (updatedGoal) {
            performedProcessing = true;

            // Making system hold current localization so planning can be done
            plannedPath.clear();
            plannedPath.push_back(lastRobotLocation);
            publishPlannedPathMarkers(plannedPath);

            // If there was a previous thread than it is nicely joined
            if (planThreadPtr && planThreadPtr->joinable())
                planThreadPtr->join();

            updatedGoal = false;

            // Spawning new thread to plan path
            planThreadPtr.reset(new std::thread(&Planner3D::plan, this));
        }

        if (!performedProcessing)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cout << "Planner3D::run() - the end!" << std::endl;
}


void Planner3D::plan() {
    ROS_INFO_STREAM("Planner3D::plan()");
    breakPlanning = false;

    // Let's clear for debugging reasons
    updatePlanningStatus(ompl::base::PlannerStatus::UNKNOWN);

    // If octree is empty then we can only assume that the path straight to goal is the best choice
    if (emptyOctree) {
        ROS_INFO_STREAM("Straight line!");
        plannedPath.clear();
        plannedPath.push_back(lastRobotLocation);
        plannedPath.push_back(goalLocation);

        return;
    }

    // Space for planning 
    space = ompl::base::StateSpacePtr(new BiasedRealVectorStateSpace(planningObjectiveZWeight));
  
    // Defining the min and max dimensions
    ompl::base::RealVectorBounds bounds(3);

    // Octree bounds
    double minX, minY, minZ, maxX, maxY, maxZ;
    octree->getMetricMin(minX, minY, minZ);
    octree->getMetricMax(maxX, maxY, maxZ);

    // Make sure robot is within these bounds
    minX = std::min(minX, lastRobotLocation[0]-1);
    maxX = std::max(maxX, lastRobotLocation[0]+1);
    minY = std::min(minY, lastRobotLocation[1]-1);
    maxY = std::max(maxY, lastRobotLocation[1]+1);
    minZ = std::min(minZ, lastRobotLocation[2]-1);
    maxZ = std::max(maxZ, lastRobotLocation[2]+1);

    bounds.setLow(0, minX); // min X
    bounds.setHigh(0, maxX); // max X
    bounds.setLow(1, minY); // min Y
    bounds.setHigh(1, maxY); // max Y
    bounds.setLow(2, minZ); // min Z
    bounds.setHigh(2, maxZ); // max Z

    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Creating information about space
    spaceInformation = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(space));

    // Problem to be solved
    problemDefinition = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(spaceInformation));

    // Setting start & end
    ompl::base::ScopedState <ompl::base::RealVectorStateSpace> start(space), goal(space);
    start->values[0] = lastRobotLocation[0];
    start->values[1] = lastRobotLocation[1];
    start->values[2] = lastRobotLocation[2];

    // If deadlock -> then plan to the goal of last successful plan
    goal->values[0] = goalLocation[0];
    goal->values[1] = goalLocation[1];
    goal->values[2] = goalLocation[2];

    problemDefinition->setStartAndGoalStates(start, goal);

    // Getting the unique access to octree
    std::shared_lock<std::shared_timed_mutex> lock(octree_mtx);

    // Checking collision
    spaceInformation->setStateValidityChecker(
            ompl::base::StateValidityCheckerPtr(new ValidityChecker(spaceInformation, robot, octreeInFCL)));

    // What do we optimize - path_length, clearance or both with weights
    problemDefinition->setOptimizationObjective(getPathLengthObjective(spaceInformation, 0.0));

    // Create a planner for the defined space. List of available solvers: http://ompl.kavrakilab.org/namespaceompl_1_1geometric.html
    planner = ompl::base::PlannerPtr(new ompl::geometric::RRTsharp(spaceInformation));

    // Setting the problem
    planner->setProblemDefinition(problemDefinition);

    // Initializing the planner
    planner->setup();

    // The result of our planning
    ompl::base::PlannerStatus plannerStatus;

    // Intial planning -- looking for the best in the set time!
    int initialPlanningIterations = planningTimeInitial / planningTerminateCondCheckInterval;
    for (int iter = 0; iter < initialPlanningIterations; iter++) {

        // Planning for the set interval
        plannerStatus = planner->solve(planningTerminateCondCheckInterval);

        // Publish planning status
        double timeLeft = (initialPlanningIterations - iter - 1) * planningTerminateCondCheckInterval;
        std::stringstream timeLeftStream;
        timeLeftStream << std::fixed << std::setprecision(2) << timeLeft;
        updatePlanningStatus(plannerStatus, "\nPhase 1 time left: " + timeLeftStream.str());
        updatePlanningStatus(plannerStatus, "\nPhase 1 time left: " + timeLeftStream.str());

        // Invalid start/end ends planning as we will not find solution
        if (plannerStatus == ompl::base::PlannerStatus::INVALID_START or
            plannerStatus == ompl::base::PlannerStatus::INVALID_GOAL) {
            return;
        }

        // The goal state changed so our planning is no longer valid
        if (updatedGoal) {
            return;
        }

        // If we already have a solution -> let's visualize it
        if (plannerStatus == ompl::base::PlannerStatus::EXACT_SOLUTION) {

            // Retrive planned path
            copyPathFromOMPL();

            if (breakPlanning) {
                breakPlanning = false;
                updatePlanningStatus(plannerStatus);
                return;
            }
            else
                publishPlannedPathMarkers(plannedPath);

        }
        else if (breakPlanning) {

            breakPlanning = false;
            return;
        }
    }

    // Phase 1 ended
    updatePlanningStatus(plannerStatus);

    // No solution in first try -> try again and accept everything!
    if (plannerStatus != ompl::base::PlannerStatus::EXACT_SOLUTION) {

        int secondaryPlanningIterations = planningTimeSecondary / planningTerminateCondCheckInterval;
        for (int iter = 0; iter < secondaryPlanningIterations; iter++) {

            // Planning for the set interval
            plannerStatus = planner->solve(planningTerminateCondCheckInterval);

            // Publish planning status
            double timeLeft = (secondaryPlanningIterations - iter - 1) * planningTerminateCondCheckInterval;
            std::stringstream timeLeftStream;
            timeLeftStream << std::fixed << std::setprecision(2) << timeLeft;
            updatePlanningStatus(plannerStatus, "\nPhase 2 time left: " + timeLeftStream.str());

            // We have exact solution so just end planning now!
            if (plannerStatus == ompl::base::PlannerStatus::EXACT_SOLUTION)
                break;

            // The goal state changed so our planning is no longer valid
            if (updatedGoal) {
                return;
            }

            if (breakPlanning) {
                breakPlanning = false;
                return;
            }
        }
    }

    // Do we have a solution?
    if (plannerStatus == ompl::base::PlannerStatus::EXACT_SOLUTION) {
        std::cout << "Planner3D::plan() - Solution was found!" << std::endl;

        // Retrive planned path
        copyPathFromOMPL();
       
        std::cout << "plannedPath.size() = " << plannedPath.size() << std::endl;

        publishPlannedPathMarkers(plannedPath);

        return;
    }

    std::cout << "Planner3D::plan() - No solution in the end" << std::endl;

    
    return;
}

void Planner3D::octreeCallback(const octomap_msgs::OctomapConstPtr octomap) {

    ROS_WARN_STREAM("Planner3D::updateOctree");

    // If we can't lock it means that planning is running
    if (octree_mtx.try_lock()) {

        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*octomap);
        octomap::OcTree* octree_map = dynamic_cast <octomap::OcTree*> (tree);
        octree.reset(octree_map);
        
        fcl::OcTree<double> *fclOctree = new fcl::OcTree<double>(octree);
        std::shared_ptr <fcl::CollisionGeometry<double>> collisionGeometry = std::shared_ptr < fcl::CollisionGeometry < double >> (fclOctree);
        octreeInFCL = std::make_shared < fcl::CollisionObject < double >> ((collisionGeometry));
        
        lastOctreeUpdate = ros::Time::now();
        octree_mtx.unlock();
    }

    // There are points in the octree
    if (emptyOctree && octree->getNumLeafNodes() > 0)
        emptyOctree = false;
}


void Planner3D::poseCallback(const nav_msgs::OdometryConstPtr &poseMsg) {
    geometry_msgs::Point posePoint = poseMsg->pose.pose.position;

    lastRobotLocation = Eigen::Vector3d(posePoint.x, posePoint.y, 0.2);
    std::cout << "[Planner3D] - new position: " << lastRobotLocation.transpose() << std::endl;
}


void Planner3D::publishPlannedPathMarkers(std::vector <Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &plannedPoints) {
    ROS_INFO_STREAM("[Planner3D::publishPlannedPath] Publishing planned path");

    // Reset visualization
    if (plannedPoints.size() == 0) {

        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        publisherPlannedPathMarkers.publish(marker);
    } else {

        visualization_msgs::Marker marker, markerSphere;

        // For visualization purposes it is drawn in the currently estimated robot pose
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::LINE_STRIP;

        marker.action = visualization_msgs::Marker::ADD;
        if (plannedPoints.size() == 0)
            marker.action = visualization_msgs::Marker::DELETEALL;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.id = 0;

        markerSphere.header.frame_id = "odom";
        markerSphere.header.stamp = ros::Time::now();
        markerSphere.type = visualization_msgs::Marker::SPHERE;
        markerSphere.action = visualization_msgs::Marker::ADD;
        if (plannedPoints.size() == 0)
            markerSphere.action = visualization_msgs::Marker::DELETEALL;
        markerSphere.pose.position.x = 0.0;
        markerSphere.pose.position.y = 0.0;
        markerSphere.pose.position.z = 0.0;
        markerSphere.pose.orientation.x = 0.0;
        markerSphere.pose.orientation.y = 0.0;
        markerSphere.pose.orientation.z = 0.0;
        markerSphere.pose.orientation.w = 1.0;
        markerSphere.scale.x = 0.25;
        markerSphere.scale.y = 0.25;
        markerSphere.scale.z = 0.25;
        markerSphere.color.a = 1.0;
        markerSphere.color.r = 0.0;
        markerSphere.color.g = 0.0;
        markerSphere.color.b = 1.0;
        markerSphere.id = 1;

        for (int i = 0; i < plannedPoints.size(); i++) {

            geometry_msgs::Point p;
            p.x = plannedPoints[i](0);
            p.y = plannedPoints[i](1);
            p.z = plannedPoints[i](2);

            marker.points.push_back(p);

            // Initial state
            if (i == 0)
                markerSphere.pose.position = p;
        }

        publisherPlannedPathMarkers.publish(markerSphere);
        publisherPlannedPathMarkers.publish(marker);
    }
}

void Planner3D::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goalMsg) {
    geometry_msgs::Point goalPoint = goalMsg->pose.position;
    ROS_INFO_STREAM("[Planner3D::goalCallback] Setting the goal as (" << goalPoint.x << ", " << goalPoint.y << ", "
                                                                        << goalPoint.z << ")");

    goalLocation = Eigen::Vector3d(goalPoint.x, goalPoint.y, 0.5);
    updatedGoal = true;
}



void Planner3D::updatePlanningStatus(const ompl::base::PlannerStatus &plannerStatus, std::string additional_info) {

    // Status
    std::string msgStr;
    if (plannerStatus == ompl::base::PlannerStatus::UNKNOWN) {
        msgStr = "New planning requested!";
    } else if (plannerStatus == ompl::base::PlannerStatus::EXACT_SOLUTION) {
        msgStr = "Exact solution was found!" + additional_info;
    } else if (plannerStatus == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
        msgStr = "Only an approximate solution was found!" + additional_info;
    } else if (plannerStatus == ompl::base::PlannerStatus::INVALID_START) {
        msgStr = "Initial position is not valid!";
    } else if (plannerStatus == ompl::base::PlannerStatus::INVALID_GOAL) {
        msgStr = "Goal position is not valid!";
    } else if (plannerStatus == ompl::base::PlannerStatus::TIMEOUT) {
        msgStr = "We need more time!";
    } else {
        msgStr = "Failed with other reason!";
    }

    // screen
//    std::cout << "Planner3D::plan() - " << msgStr << std::endl;

    // ROS to rviz
    std_msgs::String msg;
    msg.data = msgStr;
    publisherPlannerString.publish(msg);
}

void Planner3D::copyPathFromOMPL() {

    // Clear last path
    plannedPath.clear();

    // Path information
    ompl::base::PathPtr path = problemDefinition->getSolutionPath();

    // Path as ompl::geometric
    ompl::geometric::PathGeometric *pathAsGeo = problemDefinition->getSolutionPath()->as<ompl::geometric::PathGeometric>();

    // Saving the path using Eigen
    std::vector < ompl::base::State * > states = pathAsGeo->getStates();
    for (auto &s : states) {
        const ompl::base::RealVectorStateSpace::StateType *castedState = s->as<ompl::base::RealVectorStateSpace::StateType>();

        plannedPath.emplace_back(
                Eigen::Vector3d(castedState->values[0], castedState->values[1], castedState->values[2]));
    }
}
