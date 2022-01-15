#ifndef READPARAMETER_H
#define READPARAMETER_H

#include <ros/ros.h>
#include <string>

template <class T>
T readParameter(ros::NodeHandle &nh, std::string TAG, std::string name, T defaultValue) {
    T value;
    if (nh.getParam(name.c_str(), value))
        ROS_INFO_STREAM(TAG << name << " : " << value);
    else {
        ROS_ERROR_STREAM(TAG << "No value for " << name << " set -- default equal to " << value);
        value = defaultValue;
    }
    return value;
}

#endif 
