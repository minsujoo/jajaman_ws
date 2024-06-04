#include <fstream>
#include <sstream>
#include <iostream>
#include "centerline_projector/centerline_projector_node.hpp"

void ReferencePointsNode::loadReferencePoints(const std::string& csv_file)
{
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", csv_file.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream s(line);
        std::string x_str, y_str;

        if (!std::getline(s, x_str, ',') || !std::getline(s, y_str, ',')) {
            continue;
        }

        double x = std::stod(x_str);
        double y = std::stod(y_str);

        reference_points_.emplace_back(x, y);
    }

    file.close();
}
