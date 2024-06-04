#ifndef LOAD_CENTERLINE_FROM_CSV_HPP
#define LOAD_CENTERLINE_FROM_CSV_HPP

#include <vector>
#include <string>
#include <geometry_msgs/Point.h>

std::vector<geometry_msgs::Point> loadCenterlineFromCSV(const std::string& filename);

#endif
