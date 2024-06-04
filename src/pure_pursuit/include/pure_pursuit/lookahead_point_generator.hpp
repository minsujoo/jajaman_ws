#ifndef LOOKAHEAD_POINT_GENERATOR_HPP
#define LOOKAHEAD_POINT_GENERATOR_HPP

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int16.h"
#include <vector>
#include <string>

#define LOOKAHEAD 10

class PoiPlanner {
public:
    PoiPlanner(ros::NodeHandle& nh, const std::string& centerline_file);
    void run();

private:
    ros::NodeHandle& nh_;
    ros::Publisher pub_;
    ros::Subscriber index_sub_;
    std::vector<geometry_msgs::Point> centerline_;

    void indexCallback(const std_msgs::Int16::ConstPtr& msg);
};

#endif

