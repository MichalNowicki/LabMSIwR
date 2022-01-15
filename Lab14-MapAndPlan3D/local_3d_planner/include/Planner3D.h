#ifndef PLANNER3D_H
#define PLANNER3D_H

#include <iostream>

#include <octomap/octomap.h>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>

#include <ompl/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <thread>
#include <memory>

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <chrono>

#include <algorithm>

#include "ReadParameter.h"
#include "ValidityChecker.h"
#include "planningObjectives/PlanningObjectives.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>


#include <mutex>
#include <shared_mutex>

class Planner3D {

    enum Objective {
        PATH_LENGTH,
        CLEARANCE,
        BALANCED_PATH_LENGTH_AND_CLEARANCE
    };

public:

    Planner3D();
    ~Planner3D();

    // Main running thread
    void run();

    // Main planning is called there
    void plan();

    // Update octomap
    void octreeCallback(const octomap_msgs::OctomapConstPtr octomap);

    // Update pose estimate
    void poseCallback(const nav_msgs::OdometryConstPtr &poseMsg);

    // Update goal
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);

    // Publish planned path
    void publishPlannedPathMarkers(std::vector <Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &plannedPoints);

private:

    // Publishing the planning state from OMPL
    void updatePlanningStatus(const ompl::base::PlannerStatus &plannerStatus, std::string additional_info = "");

    // Getting the path as a list of 3D points
    void copyPathFromOMPL();

    // Nodehandle + TAG
    const std::string TAG = "[planner_3d] ";
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber subscriberGoal, subscriberOctree, subscriberPose;

    // Publishers
    ros::Publisher publisherPlannerString, publisherPlannedPathMarkers;

    // If new information is available from subscribers
    bool updatedGoal;
    
    // Size of the robot
    std::shared_ptr <fcl::CollisionObject<double>> robot;

    // Space and its information for planning
    ompl::base::StateSpacePtr space;
    ompl::base::SpaceInformationPtr spaceInformation;

    // Problem to solve
    ompl::base::ProblemDefinitionPtr problemDefinition;

    // Planner to be used
    ompl::base::PlannerPtr planner;

    // Octree
    std::shared_timed_mutex octree_mtx;
    std::shared_ptr <fcl::CollisionObject<double>> octreeInFCL;
    std::shared_ptr <octomap::OcTree> octree;

    // Locations of start, goal and current octree origin
    Eigen::Vector3d octreeStart, lastRobotLocation, goalLocation;

    // Last planned path
    std::vector <Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> plannedPath;

    // Thread
    std::shared_ptr <std::thread> mainThreadPtr, octreeUpdateThreadPtr, planThreadPtr;
    bool planningThreadRun;
    std::mutex octreeUpdateMtx;

    // Last octree time
    ros::Time lastOctreeUpdate;
    bool emptyOctree;

    // Planning parameters
    double planningTimeInitial, planningTimeSecondary, planningTerminateCondCheckInterval;
    double planningObjectiveZWeight;

    // End of planning
    bool breakPlanning;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif