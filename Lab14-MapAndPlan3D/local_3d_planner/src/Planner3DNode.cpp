#include <iostream>
#include "ros/ros.h"

#include "Planner3D.h"


int main(int argc, char** argv){
    ros::init( argc, argv, "planner_3d" );

    Planner3D planner3D;
    ros::spin();
}


